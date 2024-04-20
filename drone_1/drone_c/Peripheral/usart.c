/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "main.h"
/* USER CODE BEGIN 0 */
unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeReceiveBuffer_1[JudgeBufBiggestSize];
//unsigned char JudgeReceiveBuffer_2[JudgeBufBiggestSize];

uint16_t Receive_Judge_Date_Len = 0;

extern uint8_t Transmit_Pack[128];//裁判系统数据帧
#define SEND_MAX_SIZE 128
unsigned char JudgeSend[SEND_MAX_SIZE];
/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USART3 init function */

//遥控器串口初始化
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART6 init function */
void MX_UART6_Init(void)
{

  /* USER CODE BEGIN UART6_Init 0 */

  /* USER CODE END UART6_Init 0 */

  /* USER CODE BEGIN UART6_Init 1 */

  /* USER CODE END UART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART6_Init 2 */

  /* USER CODE END UART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
   if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart6_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart6_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart6_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */
		__HAL_USART_ENABLE_IT(&huart6,UART_IT_RXNE);
		__HAL_USART_ENABLE_IT(&huart6,UART_IT_IDLE);
  /* USER CODE END USART6_MspInit 1 */
  
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN UART6_MspDeInit 0 */

  /* USER CODE END UART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**UART6 GPIO Configuration
    PA1     ------> UART6_RX
    PA0-WKUP     ------> UART6_TX
    */
   HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* UART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN UART6_MspDeInit 1 */

  /* USER CODE END UART6_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
///**********************************************************************************************************
//*函 数 名: DMA2_Stream6_IRQHandler
//*功能说明: usart6 DMA发送中断
//*形    参: 无
//*返 回 值: 无
//**********************************************************************************************************/
//extern uint8_t DMAsendflag;
uint8_t DMAsendflag;
//void DMA2_Stream6_IRQHandler(void)
//{	
//    HAL_DMA_IRQHandler(&hdma_usart6_tx);
//}

/**********************************************************************************************************
*函 数 名: UART6_IRQHandler
*功能说明: usart4中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint8_t JudgeReveice_Flag;
uint8_t JudgeReveice_DMA_DIR_Flag  = 0;
//void USART6_IRQHandler(void)
//{
////	HAL_UART_IRQHandler(&huart6);
//		if(HAL_USART_GetState(&huart6) == UART_IT_IDLE)
//		{
//			HAL_UART_IDLECallback(&huart6);
//		}
//		
//}

/**********************************************************************************************************
*函 数 名: HAL_UART_TxCpltCallback
*功能说明: tx发送完成中断
*形    参: &huart6
*返 回 值: 无
**********************************************************************************************************/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        DMAsendflag = 0;
    }
}

/**********************************************************************************************************
*函 数 名: HAL_UART_IDLECallback
*功能说明: 空闲中断回调
*形    参: &huart6
*返 回 值: 无
**********************************************************************************************************/

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
			uint8_t rc_tmp=UART4->SR;
			rc_tmp=UART4->DR;//软件序列清除IDLE标志位
        uint16_t Receive_Len = 0;
        HAL_DMA_Abort(&hdma_usart6_rx);
        Receive_Len = JudgeBufBiggestSize - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
        Receive_Judge_Date_Len = Receive_Len;
        JudgeReveice_Flag = 1;
        memcpy(JudgeReceiveBuffer_1, JudgeReceiveBuffer, Receive_Len);
        HAL_DMA_Start(&hdma_usart6_rx, (uint32_t)&USART6->DR, (uint32_t)JudgeReceiveBuffer, JudgeBufBiggestSize);
    }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
