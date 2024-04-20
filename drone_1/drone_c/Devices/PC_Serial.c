#include "main.h"
#include "PC_Serial.h"

float pc_pitch,pc_yaw;

unsigned char SendToPC_Buff[PC_SENDBUF_SIZE] = {0};
unsigned char PCbuffer[PC_RECVBUF_SIZE];

PCRecvData PC_Recv;

short i,i1;
short last_shoot_flag = 0;

void PCReceive(unsigned char *PCbuffer)//辅瞄接收函数
{
		i++;
		if(PCbuffer[0] == '!' && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
		{
				if(PCbuffer[0] == '!' )
				{
					//数据解码
					memcpy(&PC_Recv, PCbuffer, PC_RECVBUF_SIZE);

					PC_Recv.pitch = PC_Recv.pitch/100.0f;
					pc_yaw = PC_Recv.yaw;		//PCyaw,pitch

					PC_Recv.shoot_flag = PC_Recv.shoot_flag - last_shoot_flag;
					last_shoot_flag = PC_Recv.shoot_flag;
					i1++;
				}
		}
}

	PCSendData PC_send_data;
//给PC发送当前机器人是哪一方，子弹速度，云台模式，YAW、PITCH轴角度
void SendtoPCPack(unsigned char *buff)
{

	
	//1字节0
	PC_send_data.start_flag = '!';
	//1字节1
	PC_send_data.robot_color = 0;
	//1字节2
	PC_send_data.shoot_level = 3;
	//1字节2
	PC_send_data.mode = 1;
	//1字节2
	PC_send_data.which_balance = 2;
	//1字节1
	PC_send_data.change_priority_flag = 1;
	
	//1字节
	PC_send_data.frame_id = 0;
	//
	//2字节 
	PC_send_data.pitch = (short)(pitch.imuAngle * 100.0f);
	//PC_send_data.pitch = pitch.Angle*100;
	//4字节 
	PC_send_data.yaw = yaw.imuAngle;
	
	
	//2字节 
	PC_send_data.crc16 = 9;
	//Append_CRC16_Check_Sum((unsigned char *)&PC_send_data, PC_SENDBUF_SIZE);
	memcpy(buff, (void *)&PC_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
	SendtoPCPack(SendToPC_Buff);	
	CDC_Transmit_FS(SendToPC_Buff,PC_SENDBUF_SIZE);
}
