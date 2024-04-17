/**********************************************************************************************************
 * @文件     uart4.c
 * @说明     uart4 f4初始化(裁判系统通信)
 * @版本  	 V2.0
 * @作者     郭嘉豪
 * @日期     2022.3
**********************************************************************************************************/




/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "main.h"
#include "uart4.h"

/* USER CODE BEGIN 0 */
unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeReceiveBuffer_1[JudgeBufBiggestSize];
//unsigned char JudgeReceiveBuffer_2[JudgeBufBiggestSize];

uint16_t Receive_Judge_Date_Len = 0;

extern uint8_t Transmit_Pack[128];//裁判系统数据帧
unsigned char JudgeSend[SEND_MAX_SIZE];
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;


/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART3 init function */



}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA1     ------> UART4_RX
    PA0-WKUP     ------> UART4_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA1_Stream4;
    hdma_uart4_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_uart4_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_uart4_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_uart4_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA1     ------> UART4_RX
    PA0-WKUP     ------> UART4_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_0);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  
}

/* USER CODE BEGIN 1 */
///**********************************************************************************************************
//*函 数 名: DMA1_Stream_IRQHandler
//*功能说明: usart2 DMA接收中断
//*形    参: 无
//*返 回 值: 无
//**********************************************************************************************************/
uint8_t JudgeReveice_Flag;
uint8_t JudgeReveice_DMA_DIR_Flag  = 0;
void DMA1_Stream2_IRQHandler(void)
{	
	//HAL_DMA_GetState();
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{ 
		JudgeReveice_DMA_DIR_Flag = DMA_GetCurrentMemoryTarget(DMA1_Stream2);
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	}
}
///**********************************************************************************************************
//*函 数 名: DMA1_Stream4_IRQHandler
//*功能说明: usart2 DMA发送中断
//*形    参: 无
//*返 回 值: 无
//**********************************************************************************************************/
extern uint8_t DMAsendflag;
void DMA1_Stream4_IRQHandler(void)
{	
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
	{
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		DMA_Cmd(DMA1_Stream4, DISABLE);			
        DMAsendflag = 0;
	}
}

void UART4_IRQHandler(void)
{
	uint16_t Receive_Len = 0;
	if(SET==USART_GetITStatus(UART4,USART_IT_IDLE))
	{
		uint8_t rc_tmp=UART4->SR;
		rc_tmp=UART4->DR;//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream2, DISABLE);
		Receive_Len = JudgeBufBiggestSize - DMA_GetCurrDataCounter(DMA1_Stream2);
		Receive_Judge_Date_Len = Receive_Len;
		JudgeReveice_Flag = 1;
		memcpy(JudgeReceiveBuffer_1,JudgeReceiveBuffer,Receive_Len);
		DMA_SetCurrDataCounter(DMA1_Stream2,JudgeBufBiggestSize);
		DMA_Cmd(DMA1_Stream2, ENABLE);
	}
	
}

/* USER CODE END 1 */


