#include "bsp_can.h"
#include "main.h"
#include "motor.h"
#include "selfcheck_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


/**********************************************************************************************************
 *函 数 名: can_filter_init
 *功能说明: can配置
 *形    参: 无
 *返 回 值: 无
 **********************************************************************************************************/
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
//CAN 2 FIFO0 接收中断	
	  can_filter_st.FilterBank = 15;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh =0x096<<5;//大符 0x096<<5;
    can_filter_st.FilterIdLow =0x094<<5; //裁判系统0x094<<5;
    can_filter_st.FilterMaskIdHigh =0x100<<5;//底盘控制反馈 0x100<<5;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
//CAN 2 FIFO1 接收中断
	  can_filter_st.FilterBank = 16;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = 0x205<<5;//YAW电机
    can_filter_st.FilterIdLow =  0x206<<5;//Pitch
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);	
//CAN 1 FIFO0 接收中断
    can_filter_st.FilterBank = 0;
		can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter_st.FilterIdHigh = 0x205<<5;//PITCH电机
    can_filter_st.FilterIdLow =  0x206<<5;//Yaw
    can_filter_st.FilterMaskIdHigh =0x200<<5;//Pitch
    can_filter_st.FilterMaskIdLow = 0x200<<5;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
		can_filter_st.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//CAN 1 FIFO1 接收中断		
		can_filter_st.FilterBank = 1;
		can_filter_st.FilterActivation = ENABLE;
		can_filter_st.FilterMode = CAN_FILTERMODE_IDLIST;
		can_filter_st.FilterScale = CAN_FILTERSCALE_16BIT;
		can_filter_st.FilterIdHigh = 0x202<<5;//摩擦轮
		can_filter_st.FilterIdLow =  0x203<<5;//摩擦轮
		can_filter_st.FilterMaskIdHigh =0x204<<5;//拨盘
		can_filter_st.FilterMaskIdLow = 0x0000;
		can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
		can_filter_st.SlaveStartFilterBank = 15;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
		
		
//CAN 1 发送中断	
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
//CAN 2 发送中断
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
} 




/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 0邮箱中断回调函数
 *形    参: 
 *返 回 值: 无
 **********************************************************************************************************/
float time = 0;
float lasttime = 0;
float interval = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	 CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		
	if (hcan->Instance == CAN1)
	{      // 处理CAN1的数据	
		switch(rx_header.StdId)
			{
				case 0x205:
				{
					yaw.MotorAngle = rx_data[0]<<8 | rx_data[1];
					yaw.MotorSpeed = rx_data[2]<<8 | rx_data[3];
					yaw.torque = rx_data[4]<<8 | rx_data[5];
				
					yaw.Angle_zeroCheck = Get_ZeroCheck_Value_Interface(&yaw.zero,yaw.MotorAngle);
					//yaw.feedback_angle = yaw.Angle_zeroCheck/8192;
					
					/* update receive time for selfcheck */
					yaw.updateTime = xTaskGetTickCount();
					break;
				}
				
				case 0x206:
				{
					pitch.MotorAngle = rx_data[0]<<8 | rx_data[1];				
					pitch.MotorSpeed = rx_data[2]<<8 | rx_data[3];
					pitch.torque = rx_data[4]<<8 | rx_data[5]; //Current				
					pitch.Angle_zeroCheck = Get_ZeroCheck_Value_Interface(&pitch.zero,pitch.MotorAngle);
					//pitch.feedback_angle = pitch.Angle_zeroCheck/8192;					
					/* update receive time for selfcheck */
					pitch.updateTime = xTaskGetTickCount();
					break;
				}
		}
			
		 time = xTaskGetTickCount();
		interval = time-lasttime;
		lasttime = time;
	}
	else if (hcan->Instance == CAN2) 
	{
	}
}
/**********************************************************************************************************
 *函 数 名: HAL_CAN_RxFifo0MsgPendingCallback
 *功能说明:FIFO 1邮箱中断回调函数
 *形    参: 
 *返 回 值: 无
 **********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)//FIFO 1邮箱中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
	__HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
		
	if (hcan->Instance == CAN1)
	{
			switch(rx_header.StdId)
			{			
				case 0x202:
				{
					frimition[0].Angle = rx_data[0]<<8 | rx_data[1];
					frimition[0].Speed = rx_data[2]<<8 | rx_data[3];
					frimition[0].torque = rx_data[4]<<8 | rx_data[5];
				
					frimition[0].Angle_zeroCheck = Get_ZeroCheck_Value_Interface(&frimition[0].zero,frimition[0].Angle);
					
					/* update receive time for selfcheck */
					frimition[0].updateTime = xTaskGetTickCount();
					break;
				}
				case 0x203:
				{
					frimition[1].Angle = rx_data[0]<<8 | rx_data[1];
					frimition[1].Speed = rx_data[2]<<8 | rx_data[3];
					frimition[1].torque = rx_data[4]<<8 | rx_data[5];
				
					frimition[1].Angle_zeroCheck = Get_ZeroCheck_Value_Interface(&frimition[1].zero,frimition[1].Angle);
					
					/* update receive time for selfcheck */
					frimition[1].updateTime = xTaskGetTickCount();
					break;
				}	

				case 0x204:
				{
					pluck.Angle = rx_data[0]<<8 | rx_data[1];
					pluck.Speed = rx_data[2]<<8 | rx_data[3];
					pluck.torque = rx_data[4]<<8 | rx_data[5];
				
					pluck.Angle_zeroCheck = Get_ZeroCheck_Value_Interface(&pluck.zero,pluck.Angle);
					
					/* update receive time for selfcheck */
					pluck.updateTime = xTaskGetTickCount();
					break;
				}
			}	
	}
	else if(hcan->Instance == CAN2)
	{
			
	}	
}
