#include "RemoteControl_Task.h"
#include "main.h"


//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700


/**************声明************/
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


short RemoteRecive_flag = 0;

//遥控器控制变量
RC_Ctl_t              RC_Ctl;

static void RemoteReceive(volatile const uint8_t *rx_buffer, RC_Ctl_t *RC);
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**************声明************/



/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


//开启一次 DMA 传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
//void DMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
//uint16_t Size)
//{
//	HAL_DMA_Start(huart->hdmatx, (uint32_t )pData, (uint32_t)&huart->Instance->DR,Size);//开启 DMA 传输
//	huart->Instance->CR3 |= USART_CR3_DMAT;//使能串口 DMA 发送
//	
//	while(1)
//	{
//		if(__HAL_DMA_GET_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7))
//		//等待 DMA2_Steam7 传输完成
//		{
//		__HAL_DMA_CLEAR_FLAG(&UART1TxDMA_Handler,
//		DMA_FLAG_TCIF3_7);//清除 DMA2_Steam7 传输完成标志
//		HAL_UART_DMAStop(&UART1_Handler);
//		//传输完成以后关闭串口 DMA
//		break;
//		}
//	}
//}




//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
							
								RemoteRecive_flag = 1;
                //记录数据接收时间
//                detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
								RemoteRecive_flag = 2;

            }
        }
    }

		RC_Ctl.updateTime = xTaskGetTickCount();
}


/**
  * @brief          遥控器协议解析
  * @param[in]      rx_buffer: 原生数据指针
  * @param[out]     RC_Ctl: 遥控器数据指
  * @retval         none
  */
static void RemoteReceive(volatile const uint8_t *rx_buffer, RC_Ctl_t *RC)
{
	if (rx_buffer == NULL)
	{
			return;
	}

  RC->rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; 
	RC->rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; 
	RC->rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;
	RC->rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff;
		
	RC->rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //LEFT 
	RC->rc.s2 = ((rx_buffer[5] >> 6)& 0x0003); //RIGHT 
	
	if((RC->rc.ch0-1024<15)&&(RC->rc.ch0-1024>-15)) RC->rc.ch0=1024;
	if((RC->rc.ch1-1024<15)&&(RC->rc.ch1-1024>-15)) RC->rc.ch1=1024;
	if((RC->rc.ch2-1024<10)&&(RC->rc.ch2-1024>-10)) RC->rc.ch2=1024;
	if((RC->rc.ch3-1024<10)&&(RC->rc.ch3-1024>-10)) RC->rc.ch3=1024;	
	
	RC->mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); 
	RC->mouse.y = rx_buffer[8] | (rx_buffer[9] << 8);
	RC->mouse.z = rx_buffer[10] | (rx_buffer[11] << 8);
	
	RC->mouse.press_l = rx_buffer[12]; 
	RC->mouse.press_r = rx_buffer[13];
	
	RC->key.w = rx_buffer[14]&0x01;
	RC->key.s = (rx_buffer[14]>>1)&0x01;
	RC->key.a = (rx_buffer[14]>>2)&0x01;
	RC->key.d = (rx_buffer[14]>>3)&0x01;
	RC->key.shift =(rx_buffer[14]>>4)&0x01;
	RC->key.ctrl = (rx_buffer[14]>>5)&0x01;
	RC->key.q = (rx_buffer[14]>>6)&0x01;
	RC->key.e = (rx_buffer[14]>>7)&0x01;	
	RC->key.r = (rx_buffer[15])&0x01;
	RC->key.f = (rx_buffer[15]>>1)&0x01;
	RC->key.g = (rx_buffer[15]>>2)&0x01; 
	RC->key.z = (rx_buffer[15]>>3)&0x01;
	RC->key.x = (rx_buffer[15]>>4)&0x01;
	RC->key.c = (rx_buffer[15]>>5)&0x01;
	RC->key.v = (rx_buffer[15]>>6)&0x01;
	RC->key.b = (rx_buffer[15]>>7)&0x01;
}


/**********************************************************************************************************
 *函 数 名: RCReceive_task
 *功能说明: 遥控器数据处理任务
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
uint32_t RC_high_water;
void RCReceive_task(void const * argument)
{
	while (1)
		{
			SendtoPC();
			/********************************* 遥控器数据处理 *****************************************************/
			if(RemoteRecive_flag == 1)			
			{ 
				RemoteReceive(sbus_rx_buf[0], &RC_Ctl);		//解码函数
			}else if(RemoteRecive_flag == 2)
			{  
				RemoteReceive(sbus_rx_buf[1], &RC_Ctl);		//解码函数
			}
			RemoteRecive_flag = 0;
			osDelay(5);
	}
}
