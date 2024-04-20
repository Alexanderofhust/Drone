#ifndef __SELFCHECK__
#define __SELFCHECK__

#include "stm32f4xx.h"
#include "gimbal_task.h"
#include "shoot_task.h"

#define SELFCHECK		 	1 //云台设备自检开启

#define Plane_ID 1

/* 位域用法: 为了节省内存空间 可以将一个数据类型拆分成位来使用 */
/* 冒号后面表示该位域所占的位数 如下:每个位域长度为2Bit */
/* 注意: 位域必须存储在同一个类型中 不能跨类型 同时也说明位域的长度不会超过所定义类型的长度
   如果一个定义类型单元里所剩空间无法存放下一个域 则下一个域应该从下一单元开始存放 */
struct compose {
	/* 注意最先定义的位域在数据的末尾(即从后向前填充) */
	unsigned int IMU_disconneced 			: 2; //错误代码:0x0001
	unsigned int RC_disconneced 			: 2; //错误代码:0x0040

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
	/* union中的内容指向同一个地址 */
	union
	{
		unsigned short droneState;//代替composeState
		struct compose composeState;//状态标志位
	}drone;
	
	uint8_t Gimbal_poweroff;
	uint8_t AimAssist;
	uint8_t PC_Received;
	short ControlMode;//遥控模式
	short MotorMode;
	short ShootMode;//射击模式
}state;

extern struct state droneState;
void SelfcheckTask(void const *pvParameters);


#endif


/* end */
















