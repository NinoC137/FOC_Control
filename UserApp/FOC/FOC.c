#include "FOC.h"
#include "cmsis_os.h"

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float shaft_angle = 0.0f, open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;
float voltage_power_supply = 12.0f;
//float Ualpha = 0.0f, Ubeta = 0.0f;
//float Ua, Ub, Uc, dc_a, dc_b, dc_c;

int PP, DIR;

//传感器数值
float angle_pi;
float angle_f;

LowPassFilter filter = {.Tf=0.01f, .y_prev=0.0f}; //Tf=10ms
PIDController pid_controller = {.P=0.2f, .I=0.1f, .D=0.0f, .output_ramp=100.0f, .limit=6, .error_prev=0, .output_prev=0, .integral_prev=0};

//PID
PIDController *vel_loop_M0;
PIDController *angle_loop_M0;

//===================================PID 设置函数=========================================
////速度PID
//void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp)   //M0角度环PID设置
//{
//    vel_loop_M0->P = P;
//    vel_loop_M0->I = I;
//    vel_loop_M0->D = D;
//    vel_loop_M0->output_ramp = ramp;
//}
//
////角度PID
//void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp)   //M0角度环PID设置
//{
//    angle_loop_M0->P = P;
//    angle_loop_M0->I = I;
//    angle_loop_M0->D = D;
//    angle_loop_M0->output_ramp = ramp;
//}

//M0速度PID接口
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
//    Motor_1.Error = error;
    Motor_1.Error = Low_Pass_Filter(&lpf_Motor1_error, error, 0.7f);
    return Position_Pid_Calculate(&Motor_1);
}

//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error) {
    Motor_1.Error = Low_Pass_Filter(&lpf_Motor1_error, error, 0.7f);
    return Position_Pid_Calculate(&Motor_1);
}
//=====================================================================================

void FOC_Vbus(float _Vbus) {
    voltage_power_supply = _Vbus;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

//    struct LowPassFilter filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms
//    struct PIDController pid_controller = {.P=0.5,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

}

void FOC_alignSensor(int _PP, int _DIR) {
    PP = _PP;
    DIR = _DIR;

    setTorque(3, _3PI_2);  //起劲
    HAL_Delay(1000);
    i2c_mt6701_get_angle(&angle_pi, &angle_f); //更新传感器数值
    zero_electric_angle = _electricalAngle(angle_pi, (int) PP);
    setTorque(0, _3PI_2);  //松劲（解除校准）

    uart_printf("zero electric angle: %f\r\n", zero_electric_angle);
}

float DFOC_M0_Velocity() {
    static float angle_now, angle_old;
    static long sampleTimeStamp;
    i2c_mt6701_get_angle(&angle_pi, &angle_f); //更新传感器数值
    sampleTimeStamp = osKernelSysTick();
    angle_now = angle_pi;

    float delta_angle = angle_now - angle_old;
    if (delta_angle >= 1.6 * M_PI) {
        delta_angle -= 2.0f * M_PI;
    }
    if (delta_angle <= -1.6 * M_PI) {
        delta_angle += 2.0f * M_PI;
    }

    float vel_speed_ori = delta_angle / 3e-3;  //采样时间以3ms为单位, 乘1e3后为每秒的角速度

    angle_old = angle_now;

    float vel_M0_flit = LowPassFilter_process(&filter, DIR * vel_speed_ori);
//    Motor_1.Actual = vel_M0_flit;

    return vel_M0_flit;
}

float DFOC_M0_Angle() {
    i2c_mt6701_get_angle(&angle_pi, &angle_f);
    return DIR * angle_pi;
}

//电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
//    return (shaft_angle * pole_pairs);
    return _normalizeAngle(((float)(DIR * pole_pairs)*shaft_angle)-zero_electric_angle);
}

//角度归一化
float _normalizeAngle(float angle) {
    float a = fmod(angle, 2*PI);
    return a >= 0 ? a : (a + 2*PI);
}

//输出PWM
void setPWM(float Ua, float Ub, float Uc) {
    // 限制上限
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);

    //计算占空比, 并限制其在0~1
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    //写入PWM通道
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, dc_a * htim3.Init.Period);
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, dc_b * htim4.Init.Period);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, dc_c * htim5.Init.Period);
}

void setTorque(float Uq, float angle_el) {
    Uq = _constrain(Uq, -(voltage_power_supply) / 3, (voltage_power_supply) / 3);   //力矩限制, 限定为上限电压的五分之一, 保证安全性

    angle_el = _normalizeAngle(angle_el);

    // 帕克逆变换
    float Ualpha = -Uq * (float) sin(angle_el);
    float Ubeta = Uq * (float) cos(angle_el);

    //克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (float) (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (float) (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

//设置相电压
void setPhaseVoltage(float Uq, float Ud, float angle_elec) {
    angle_elec = _normalizeAngle(angle_elec);
    // 帕克逆变换
    float Ualpha = -Uq * sin(angle_elec);
    float Ubeta = Uq * cos(angle_elec);

    // 克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

//开环速度函数
float velocityOpenLoop(float target_velocity) {
    unsigned long now_us = HAL_GetTick();  //获取从开启芯片以来的微秒数，它的精度是 1ms

    //计算当前每个Loop的运行时间间隔
    float Ts = (now_us - open_loop_timestamp) * 1e-3f;

    //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
//    if (Ts <= 0 || Ts > 0.5f) Ts = 5 * 1e-2f;


    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = voltage_power_supply / 8.0f;

    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));

    open_loop_timestamp = now_us;  //用于计算下一个时间间隔

    return Uq;
}

//================简易接口函数================
void FOC_M0_set_Velocity_Angle(float Target) {
    i2c_mt6701_get_angle(&angle_pi, &angle_f);
    float angle_error = Target / 180.0f * M_PI - angle_pi * DIR;

    if(angle_error > M_PI){
        angle_error -= M_PI_2;
    }

    setTorque(_constrain( DFOC_M0_ANGLE_PID(angle_error) * 180.0f / PI , Motor_1.OutputMin, Motor_1.OutputMax),
    _electricalAngle(angle_pi, PP));   //角度闭环
}

void FOC_M0_setVelocity(float Target) {
    Motor_1.Target = Target;
    setTorque(DFOC_M0_VEL_PID((Target - DFOC_M0_Velocity()) * 180 / PI), _electricalAngle(angle_pi, PP));   //速度闭环
}

void FOC_M0_set_Force_Angle(float Target)   //力位
{
    setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI), _electricalAngle(angle_pi, PP));
}

void FOC_M0_setTorque(float Target) {
    setTorque(Target, _electricalAngle(Motor_1.Error, PP));
}
