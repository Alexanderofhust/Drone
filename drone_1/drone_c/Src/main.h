#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#define Robot_ID 1

//Peripheral
#include "math_cal.h"
#include "tim.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "IMU.h"
#include "usb_device.h"

//DEVICE
#include "motor.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_conf.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "PC_Serial.h"
	
//BSP
#include "bsp_delay.h"
#include "bsp_can.h"
#include "bsp_PWM.h"
#include "counter.h"

//MYTASK
#include "selfcheck_task.h"
#include "RemoteControl_Task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "ActionControl_Task.h"
#include "INS_task.h"
// #include "debug_task.h"

#define POWER_OFF 0
#define CHARGE_ENABLE 1




void BSP_Init(void);
void Robot_Init(void);
void Offline_Check_task(void *pvParameters);
void Cmera_rising_edge(int *state,int cur_num);

void Error_Handler(void);

extern osThreadId RCReceive_task_handle;


#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
#define ABS(x) ((x)>0? (x):(-(x))) 
/* USER CODE END Private defines */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
