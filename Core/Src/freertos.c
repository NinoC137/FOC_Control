/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//uart input
#define UARTBUFFER 100
uint8_t uartBuffer[UARTBUFFER];
float reformat[2];
//hall sensor angle data
extern float angle_pi;
extern float angle_f;
extern float zero_electric_angle;
//Motor Mode
uint8_t ModeFlag;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationTickHook(void);

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook(void) {
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void FOCTask(void const *argument) {
    int Motor_PP = 7;
    int sensorDir = 1;

    FOC_Vbus(12.0f);
    FOC_alignSensor(Motor_PP, sensorDir);

    Pid_Value_Init();
    Motor_1.OutputMax = 3.0;
    Motor_1.OutputMin = -3.0;
    Motor_1.IntegralMax = 1.0f;

    for (;;) {
        switch (ModeFlag) {
            case 1:
                if (reformat[1] > 30 || reformat[1] < -30) {
                    uart_printf("argument error! Set speed to 1 rad/s\r\n");
                    reformat[1] = 1.00f;
                } else
                    velocityOpenLoop(reformat[1]);
                break;

            case 2:
                if (reformat[1] > 30 || reformat[1] < -30) {
                    uart_printf("argument error! Set speed to 1 rad/s\r\n");
                    reformat[1] = 1.00f;
                } else
                    FOC_M0_setVelocity(reformat[1]);
                break;

            case 3:
                FOC_M0_set_Velocity_Angle(reformat[1]);
                uart_printf("%f,%f\r\n", angle_pi, reformat[1]/180.0f*_PI);
                break;

            default:
                break;
        }
        osDelay(1);
    }
}

void SensorTask(void const *argument) {

    for (;;) {
//        i2c_mt6701_get_angle(&angle, &angle_f);
//        uart_printf("hall data: %f\r\n", angle_f);

//        uart_printf("Actual Speed: %f\r\n", Motor_1.Actual);
//        uart_printf("Sensor Angle: %f\r\n", angle_f);

        osDelay(500);
    }
}

void ModeSwitchTask(void const *argument) {
    for (;;) {
        osDelay(500);
    }
}

void UARTTask(void const *argument) {
    int i;
    int array_empty_flag = 1;

    uart_printf("**********************************************************************   \r\n");
    uart_printf("Nino FOC      ---     BLDC   \r\n");
    uart_printf("input example: \r\n");
    uart_printf("               x:+001,y:+020,\r\n");
    uart_printf("Motor Mode switch:   \r\n");
    uart_printf("Mode 1: OpenLoop\t\t\targument: speed(-30~30)   \r\n");
    uart_printf("Mode 2: Feedback Speed Control\t\targument: speed(-30~30)   \r\n");
    uart_printf("Mode 3: Angle Control\t\t\targument: angle(0~2pi)   \r\n");
    uart_printf("Mode 4: Gear Mode \t\t\targument: Gear Number(2~6)   \r\n");
    uart_printf("Mode 5: Without damp\t\t\targument: NULL   \r\n");
    uart_printf("Mode 6: Damp\t\t\t\targument: damp value(1~10)   \r\n");
    uart_printf("**********************************************************************   \r\n");

    for (;;) {
        HAL_UART_Receive_DMA(&huart1, &uartBuffer[0], UARTBUFFER);
        for (i = 0; i < UARTBUFFER; i++) {
            if (uartBuffer[i] != 0) {
                array_empty_flag = 0;
                break;
            }
        }
        if (array_empty_flag == 0) {
            array_empty_flag = 1;

//            uart_printf("User String: %s\r\n", uartBuffer);

            /*
             * example:
             * input: x:+001,y:-010,
             * output: reformat[0] = 1, reformat[1] = -10
             * */
            portENTER_CRITICAL();
            ReformatBuffer(uartBuffer, reformat);
            ModeFlag = (uint8_t) reformat[0];

            uart_printf("Mode: %d\targument: %0.2f\r\n", ModeFlag, reformat[1]);
            reformat[1] = reformat[1] / 180.0f * M_PI;  //转化为弧度制单位
            portEXIT_CRITICAL();

            memset(uartBuffer, 0, UARTBUFFER);
            HAL_UART_DMAStop(&huart1);
        }
        osDelay(100);
    }
}
/* USER CODE END Application */
