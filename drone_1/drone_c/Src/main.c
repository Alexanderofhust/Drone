#include "main.h"
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);


int main(void)
{

		BSP_Init();
	  delay_ms(100);
  	MX_FREERTOS_Init();
	  osKernelStart();
    while (1)
    {
    }
}


void BSP_Init(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();  //gpio初始化
	MX_DMA_Init();
	MX_SPI1_Init();//陀螺仪
	MX_TIM5_Init();//led RGB
	MX_I2C3_Init();//磁力计
	MX_TIM10_Init();//pwm控制温度
	MX_TIM3_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART3_UART_Init();//遥控
	MX_USB_DEVICE_Init();//辅瞄串口
	MX_UART6_Init();//裁判系统.
	

	MX_TIM2_Init();
	
	remote_control_init();
	
	delay_init();
	can_filter_init();
	

	DWT_Init(168);  //DWT初始化//获取cpu运行时间

	//陀螺仪初始化//1为在线标定，0为离线标定
	
	while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR){}

	MX_TIM4_Init();
	//开启定时器
    HAL_TIM_Base_Start(&htim4);
	//开启PWM通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		
}
//系统时钟初始化
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
  /* USER CODE END Error_Handle
		r_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
