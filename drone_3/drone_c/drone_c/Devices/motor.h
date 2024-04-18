#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "pid.h"
#include "zerocheck.h"
#include "feedforward.h"
/** 
  * ���ֵ��ΧΪ -30000~30000
	
  * ID ���õķ�ΧΪ1-8
  * ���͵ı�ʶ�� 0x200 --- ��ӦidΪ 1-4
  * DATA[0] id-1 ��8λ  
  * DATA[1] id-1 ��8λ
  * DATA[2] id-2 ��8λ
  * DATA[3] id-2 ��8λ
  * DATA[4] id-3 ��8λ
  * DATA[5] id-3 ��8λ
  * DATA[6] id-4 ��8λ
  * DATA[7] id-4 ��8λ
  * ���͵ı�ʶ�� 0x1FF --- ��ӦidΪ 5-7
  * DATA[0] id-5 ��8λ  
  * DATA[1] id-5 ��8λ
  * DATA[2] id-6 ��8λ
  * DATA[3] id-6 ��8λ
  * DATA[4] id-7 ��8λ
  * DATA[5] id-7 ��8λ
  * DATA[6] id-8 ��8λ
  * DATA[7] id-8 ��8λ
	
  * ����id 0x20+id(��idΪ1����ʶ��Ϊ 0x201)
  * ��������
  * DATA[0] ת�ӻ�е�Ƕȸ�8λ
  * DATA[1] ת�ӻ�е�Ƕȵ�8λ
  * DATA[2] ת��ת�ٸ�8λ
  * DATA[3] ת��ת�ٵ�8λ
  * DATA[4] ʵ�����ת�ظ�8λ
  * DATA[5] ʵ�����ת�ص�8λ
	
  * ����Ƶ�� 1kHZ
  * ת�ӻ�е�Ƕȷ�Χ 0~8191
  * ���ٱ� 36:1 
	*/
  
struct GM6020
{
	uint8_t id;
	uint32_t sendId;
	uint32_t receiveId;
	
	//���ֵ
	uint16_t MotorAngle;
	float Angle_zeroCheck;
	int16_t MotorSpeed;
	int16_t torque;//����

	//����������
	float imuSpeed;
	float imuAngle;
	
	//��������
  float autoAngle;
  
	//ʵ��ʹ�õ�ֵ
	float Speed;
	float Angle;
	
	
	Pid_Typedef SpeedPID[2];
	Pid_Typedef PositionPID[2];
	Pid_Typedef torquePID[2];
	

	ZeroCheck_Typedef zero;
	
	FeedForward_Typedef AngleFF,SpeedFF;

	uint32_t updateTime;//����ʱ��
	uint8_t Ever_Lost;
};

/**
  * @note  
  * ID ���õķ�ΧΪ1-8
  * ���͵ı�ʶ�� 0x200 ---��ӦidΪ 1-4
  * ���ֵ��ΧΪ -10000~10000
  * DATA[0] id-1 ��8λ
  * DATA[1] id-1 ��8λ
  * DATA[2] id-2 ��8λ
  * DATA[3] id-2 ��8λ
  * DATA[4] id-3 ��8λ
  * DATA[5] id-3 ��8λ
  * DATA[6] id-4 ��8λ
  * DATA[7] id-4 ��8λ
  * ���͵ı�ʶ�� 0x1ff ---��ӦidΪ 5-8
  * DATA[0] id-5 ��8λ  
  * DATA[1] id-5 ��8λ
  * DATA[2] id-6 ��8λ
  * DATA[3] id-6 ��8λ
  * DATA[4] id-7 ��8λ
  * DATA[5] id-7 ��8λ
  * DATA[6] id-8 ��8λ
  * DATA[7] id-8 ��8λ
  * ����id 0x20+id(��idΪ1����ʶ��Ϊ 0x201)
  * ��������
  * DATA[0] ת�ӻ�е�Ƕȸ�8λ
  * DATA[1] ת�ӻ�е�Ƕȵ�8λ
  * DATA[2] ת��ת�ٸ�8λ
  * DATA[3] ת��ת�ٵ�8λ
  * DATA[4] ʵ�����ת�ظ�8λ
  * DATA[5] ʵ�����ת�ص�8λ
  * ����Ƶ�� 1kHZ
  * ת�ӻ�е�Ƕȷ�Χ 0~8191
  * ���ٱ� 19:1
  */
  
struct M2006
{
	uint8_t id;
	uint32_t sendId;
	uint32_t receiveId;
	
	uint16_t Angle; //0-8191
	int32_t Angle_zeroCheck;
	int16_t Speed;
	int16_t torque;//����
	
	Pid_Typedef SpeedPID;
	Pid_Typedef PositionPID;
	Pid_Typedef TorquePID;
	

	ZeroCheck_Typedef zero;
	uint32_t updateTime;//�Լ�
	
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

