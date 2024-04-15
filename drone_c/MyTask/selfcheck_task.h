#ifndef __SELFCHECK__
#define __SELFCHECK__

#include "stm32f4xx.h"
#include "gimbal_task.h"
#include "shoot_task.h"

#define SELFCHECK		 	1 //��̨�豸�Լ쿪��

#define Plane_ID 1

/* λ���÷�: Ϊ�˽�ʡ�ڴ�ռ� ���Խ�һ���������Ͳ�ֳ�λ��ʹ�� */
/* ð�ź����ʾ��λ����ռ��λ�� ����:ÿ��λ�򳤶�Ϊ2Bit */
/* ע��: λ�����洢��ͬһ�������� ���ܿ����� ͬʱҲ˵��λ��ĳ��Ȳ��ᳬ�����������͵ĳ���
   ���һ���������͵�Ԫ����ʣ�ռ��޷������һ���� ����һ����Ӧ�ô���һ��Ԫ��ʼ��� */
struct compose {
	/* ע�����ȶ����λ�������ݵ�ĩβ(���Ӻ���ǰ���) */
	unsigned int IMU_disconneced 			: 2; //�������:0x0001
	unsigned int RC_disconneced 			: 2; //�������:0x0040

	unsigned int yaw_disconneced 			: 2; 
	unsigned int pitch_disconneced 		: 2;
    
	unsigned int Pluck_disconneced 		: 2;
	
} __attribute__ ((__packed__));


#define Lost 10
#define Rc   11
#define Rc_Shoot   12
#define Auto 13

#define Imu 0
#define Motor 1

#define stop_shoot 20
#define single_shoot 21
#define cont_shoot 22


typedef struct state
{
	/* union�е�����ָ��ͬһ����ַ */
	union
	{
		unsigned short droneState;//����composeState
		struct compose composeState;//״̬��־λ
	}drone;
	
	uint8_t Gimbal_poweroff;
	uint8_t AimAssist;
	uint8_t PC_Received;
	short ControlMode;//ң��ģʽ
	short MotorMode;
	short ShootMode;//���ģʽ
}state;

extern struct state droneState;
void SelfcheckTask(void const *pvParameters);


#endif


/* end */
















