# FOC_Control

在这一项目中, 我将使用霍尔+BLDC电机进行有感FOC驱动

## 开环驱动
调用velocityOpenLoop即可, 参数为预期速度

## 角度环闭环
调用FOC_M0_set_Velocity_Angle函数即可, 传参为预期角度(0~360度)

疑问点: 有时角度环闭环时, 电机会停在某个角度, 常见的约为220度左右, 原因未知.

此处依旧可以产生力矩, 但无法转动角度