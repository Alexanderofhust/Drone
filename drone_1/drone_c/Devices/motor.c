/* include */
#include "motor.h"
#include "main.h"


/* define */

/* declare */

//extern IMU IMUReceive;

extern float new_value; 
extern float old_value; 

extern float yaw_ffout,pitch_ffout;

extern state dronestate;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

struct GM6020 yaw;//云台电机yaw轴,yaw为航向
struct GM6020 pitch;//云台电机pitch轴,pitch为俯仰角
struct M2006  pluck;//电机
struct M2006  frimition[2];



//Yaw和Pitch发送函数
void GM6020_Send_can1(int16_t yawsend,int16_t pitchsend)
{
	/* PID */
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Can_send;
	Can_send.StdId=0x1FF;
	Can_send.IDE=CAN_ID_STD;
	Can_send.RTR=CAN_RTR_DATA;
	Can_send.DLC=0x08;
	
	static uint8_t tx_message[8];

	tx_message[0] =	yawsend>>8&0xff;
	tx_message[1] =	yawsend&0xff;
	tx_message[2] = pitchsend>>8&0xff;
	tx_message[3] = pitchsend&0xff;
	
	
	HAL_CAN_AddTxMessage(&hcan1, &Can_send, tx_message, &send_mail_box);
}

/* M2006 send data */
void M2006_Send_Can1(int16_t frimitiondata0, int16_t frimitiondata1, int16_t pluckdata)
{
	
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Can_send;
	Can_send.StdId=0x200;
	Can_send.IDE=CAN_ID_STD;
	Can_send.RTR=CAN_RTR_DATA;
	Can_send.DLC=0x08;
	


	/* 发送数据赋值 */
	static uint8_t tx_message[8];
	tx_message[2] = frimitiondata0>>8&0xff;
	tx_message[3] = frimitiondata0&0xff;
	tx_message[4] = frimitiondata1>>8&0xff;
	tx_message[5] = frimitiondata1&0xff;
	tx_message[6] = pluckdata>>8&0xff;
	tx_message[7] = pluckdata&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &Can_send, tx_message, &send_mail_box);
	
}


/* end */
