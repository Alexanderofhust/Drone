#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "pid.h"
#include "zerocheck.h"
#include "feedforward.h"
/** 
  * 输出值范围为 -30000~30000
	
  * ID 设置的范围为1-8
  * 发送的标识符 0x200 --- 对应id为 1-4
  * DATA[0] id-1 高8位  
  * DATA[1] id-1 低8位
  * DATA[2] id-2 高8位
  * DATA[3] id-2 低8位
  * DATA[4] id-3 高8位
  * DATA[5] id-3 低8位
  * DATA[6] id-4 高8位
  * DATA[7] id-4 低8位
  * 发送的标识符 0x1FF --- 对应id为 5-7
  * DATA[0] id-5 高8位  
  * DATA[1] id-5 低8位
  * DATA[2] id-6 高8位
  * DATA[3] id-6 低8位
  * DATA[4] id-7 高8位
  * DATA[5] id-7 低8位
  * DATA[6] id-8 高8位
  * DATA[7] id-8 低8位
	
  * 反馈id 0x20+id(如id为1，标识符为 0x201)
  * 反馈内容
  * DATA[0] 转子机械角度高8位
  * DATA[1] 转子机械角度低8位
  * DATA[2] 转子转速高8位
  * DATA[3] 转子转速低8位
  * DATA[4] 实际输出转矩高8位
  * DATA[5] 实际输出转矩低8位
	
  * 发送频率 1kHZ
  * 转子机械角度范围 0~8191
  * 减速比 36:1 
	*/
  
struct GM6020
{
	uint8_t id;
	uint32_t sendId;
	uint32_t receiveId;
	
	//电机值
	uint16_t MotorAngle;
	float Angle_zeroCheck;
	int16_t MotorSpeed;
	int16_t torque;//力矩

	//陀螺仪数据
	float imuSpeed;
	float imuAngle;
	
	//锟斤拷锟斤拷锟斤拷锟斤拷
  float autoAngle;
  
	//实际使用的值
	float Speed;
	float Angle;
	
	
	Pid_Typedef SpeedPID[2];
	Pid_Typedef PositionPID[2];
	Pid_Typedef torquePID[2];
	

	ZeroCheck_Typedef zero;
	
	FeedForward_Typedef AngleFF,SpeedFF;

	uint32_t updateTime;//更新时间
	uint8_t Ever_Lost;
};

/**
  * @note  
  * ID 设置的范围为1-8
  * 发送的标识符 0x200 ---对应id为 1-4
  * 输出值范围为 -10000~10000
  * DATA[0] id-1 高8位
  * DATA[1] id-1 低8位
  * DATA[2] id-2 高8位
  * DATA[3] id-2 低8位
  * DATA[4] id-3 高8位
  * DATA[5] id-3 低8位
  * DATA[6] id-4 高8位
  * DATA[7] id-4 低8位
  * 发送的标识符 0x1ff ---对应id为 5-8
  * DATA[0] id-5 高8位  
  * DATA[1] id-5 低8位
  * DATA[2] id-6 高8位
  * DATA[3] id-6 低8位
  * DATA[4] id-7 高8位
  * DATA[5] id-7 低8位
  * DATA[6] id-8 高8位
  * DATA[7] id-8 低8位
  * 反馈id 0x20+id(如id为1，标识符为 0x201)
  * 反馈内容
  * DATA[0] 转子机械角度高8位
  * DATA[1] 转子机械角度低8位
  * DATA[2] 转子转速高8位
  * DATA[3] 转子转速低8位
  * DATA[4] 实际输出转矩高8位
  * DATA[5] 实际输出转矩低8位
  * 发送频率 1kHZ
  * 转子机械角度范围 0~8191
  * 减速比 19:1
  */
  
struct M2006
{
	uint8_t id;
	uint32_t sendId;
	uint32_t receiveId;
	
	uint16_t Angle; //0-8191
	int32_t Angle_zeroCheck;
	int16_t Speed;
	int16_t torque;//锟斤拷锟斤拷
	
	Pid_Typedef SpeedPID;
	Pid_Typedef PositionPID;
	Pid_Typedef TorquePID;
	

	ZeroCheck_Typedef zero;
	uint32_t updateTime;//锟皆硷拷
	
	uint8_t Ever_Lost;
};

extern struct GM6020 yaw;
extern struct GM6020 pitch;
extern struct M2006  pluck;
extern struct M2006  frimition[2];

void GM6020_Send_can1(int16_t yawsend,int16_t pitchsend);
void M2006_Send_Can1(int16_t frimitiondata0, int16_t frimitiondata1, int16_t pluckdata);

#endif // __MOTOR_H__


/* end */

