/* include */
#include "shoot_task.h"
#include "main.h"

/* define */
#define Onegrid 37314
#define Ctn_Interval_Time 5  //������ʱ
#define Sig_Interval_Time 2000  //������ʱ

#define K1  0.9995 // ��ͨ�˲�����
#define A  0.5	  // �޷�����

uint32_t interval_time = 0;


int8_t shoot_flag = 0;

short FrictionWheel_speed_stable1; // 15000;// frictionwheel speed
short FrictionWheel_speed_stable2;

float pluckCurrent;
float frimition0Current;
float frimition1Current;

short friction_speed_last[2] = {0, 0};


float new_value = 0;
float old_value = 0;

/* declare */
static void StopShoot(void);
static void shoot_config(void);
static void Plane_ID_Shoot_Config(void);

float m=4;
float lastPos,deltaPos,pos;
int flag = 0;
int teset1 = 50;
int isPluckStuck(void)
{
	
	static int skip = 20;
	static float pos = 0;
	deltaPos = pluck.Angle_zeroCheck - lastPos;
	//�жϿ���
	if(deltaPos < teset1 && droneState.ShootMode == cont_shoot && flag != 1)
	{
		flag = 1;
		pos = pluck.Angle_zeroCheck-Onegrid;
	}
	else 
	{	
		flag = 0;
	}
	
	lastPos = pluck.Angle_zeroCheck;
	if(flag)
	{
		if(m!=0)
		{
			pluck.PositionPID.SetPoint = pos;
			return 1;
			m--;
		
		}
	}
	else
	{
		m = 4;
		flag = 0;
		return 0;
		
	}
}


void RcShoot(void)
{
	static int pluckPos_last = 0;
	static float pluckTargetPos;
	shoot_flag = 1;
	if(!isPluckStuck())
	{
		switch (droneState.ShootMode)
		{
			case single_shoot:
				//�����߼�
				if ((xTaskGetTickCount()-interval_time) > Sig_Interval_Time)
				{
					pluckPos_last = pluck.Angle_zeroCheck;
					pluckTargetPos = pluckPos_last + Onegrid;

					pluck.PositionPID.SetPoint = pluckTargetPos;
					interval_time = xTaskGetTickCount();
				}
				break;

			case cont_shoot:
				//�����߼�			
				pluckPos_last = pluck.Angle_zeroCheck;
				pluckTargetPos = pluckPos_last + 0.8*Onegrid;
				
				pluck.PositionPID.SetPoint = pluckTargetPos;
				break;
		}	
	//isPluckStuck();
	}
}

void AutoShoot(void)
{

}

void StopShoot(void)
{
	RC_Ctl.rc.ch3 = 1024;
	shoot_flag = 0;
	pluck.PositionPID.SetPoint = pluck.Angle_zeroCheck;
	pluckCurrent = 0;
}


void shootMotor_control(void)
{
	if(droneState.ControlMode != Lost)
	{
		pluck.PositionPID.ActualValue = pluck.Angle_zeroCheck;
		pluck.SpeedPID.ActualValue = pluck.Speed;

		pluckCurrent = PID_Concatenation_cal(&pluck.PositionPID, &pluck.SpeedPID);
		pluckCurrent = LIMIT_MAX_MIN(pluckCurrent, 12000, -12000);
	}

	/* friction wheel speed set */
	if (shoot_flag == 1)
	{
		frimition[0].SpeedPID.SetPoint = FrictionWheel_speed_stable1;
		frimition[1].SpeedPID.SetPoint = -FrictionWheel_speed_stable2;
	}
	else
	{
		frimition[0].SpeedPID.SetPoint = (K1 * (friction_speed_last[0]) + (1 - K1) * 0); // ��ͨ�˲�����,��Ȼ���綯�ƴ��ز���ϵͳ��ϵ�
		frimition[1].SpeedPID.SetPoint = (K1 * (friction_speed_last[1]) + (1 - K1) * 0);
	}
	friction_speed_last[0] = frimition[0].SpeedPID.SetPoint;
	friction_speed_last[1] = frimition[1].SpeedPID.SetPoint;


	// Ħ�����ٶȵ�������
	frimition[0].SpeedPID.ActualValue = frimition[0].Speed;
	frimition[1].SpeedPID.ActualValue = frimition[1].Speed;

	frimition0Current = PID_Calc(&frimition[0].SpeedPID);
	frimition1Current = PID_Calc(&frimition[1].SpeedPID);

	frimition0Current = LIMIT_MAX_MIN(frimition0Current, 16384, -16384);
	frimition1Current = LIMIT_MAX_MIN(frimition1Current, 16384, -16384);
	
	
	M2006_Send_Can1(frimition0Current, frimition1Current, pluckCurrent);
}
/*
 *    			 S2							        S1
 *  1 --> 		��           1 --> ����ģʽ
 *
 *  3 --> �������         3 --> ����ģʽ
 *
 *  2 --> ����ģʽ         2 --> ң��ģʽ
 *
 */

void ShootTask(void const *pvParameters)
{
	shoot_config();
	Plane_ID_Shoot_Config();

	/* ������ʱ */
	static short powerOff_delay = 0;
	RC_Ctl.rc.ch3 = 1024;
	for (;;)
	{
		/* �豸�Լ�������״̬������LOST����------���� */
		//pluck_check();

		//		if(droneState.drone.composeState.Pluck_disconneced ==1)
		//		{
		//			pluck.PositionPID.SetPoint = pluck.Angle_zeroCheck;
		//			shoot_flag = 0;
		//		}

		switch (droneState.ControlMode)
		{
		case Lost:
			StopShoot();
			break;

		case Rc_Shoot:		
			RcShoot();
			break;

		case Auto:
			AutoShoot();
			break;

		default:
			StopShoot();
		}

		

		/* control pluck motor */

		taskENTER_CRITICAL();
		shootMotor_control(); // ����������
		taskEXIT_CRITICAL();

		/* ���������������� �ָ�����PID���� */
		if (powerOff_delay > 0)
		{
			shoot_PID_init();
			powerOff_delay = 0;
		}

		osDelay(1);
	}
}




/* ������������ǰ�������(��ʱ) */
unsigned char shoot_ProTerminate(void)
{
	static short delay = 0;
	if (delay < 20)
	{
		delay++;
		return 0;
	}
	else
	{
		delay = 0;
		return 1;
	}
}

/* �������������� */
static void shoot_config(void)
{
	// ����
	pluck.zero.CountCycle = 8191; // pluck���һȦΪ8192

	pluck.receiveId = 0x201; // PLUCK���̵�����ID 201
	ZeroCheck_Init_Interface(&pluck.zero, 8191, pluck.Angle);
	pluck.PositionPID.SetPoint = pluck.Angle_zeroCheck; // �趨��ʼ��λ�ýǶ�ֵ

	
	
	frimition[0].receiveId = 0x202;		// ID
	frimition[0].SpeedPID.SetPoint = 0; // ��ʼ��ֵ = 0
	frimition[1].receiveId = 0x203;		// ID
	frimition[1].SpeedPID.SetPoint = 0; // ��ʼ��ֵ = 0

	shoot_PID_init();
}

void shoot_PID_init(void)
{

	
	pluck.PositionPID.P = 0.3;
	;
	pluck.PositionPID.I = 0.01;
	pluck.PositionPID.D = 0;
	pluck.PositionPID.IMax = 20000;
	pluck.PositionPID.OutMax = 20000;

	pluck.SpeedPID.P = 10;
	pluck.SpeedPID.I = 0.5;
	pluck.SpeedPID.D = 0;
	pluck.SpeedPID.IMax = 10000;
	pluck.SpeedPID.OutMax = 15000;

	// Ħ����1 �ٶ�PID��ʼ��
	frimition[0].SpeedPID.P = 10;
	frimition[0].SpeedPID.I = 0;
	frimition[0].SpeedPID.D = 0;
	frimition[0].SpeedPID.IMax = 700;
	frimition[0].SpeedPID.OutMax = 10000;

	// Ħ����2 �ٶ�PID��ʼ��
	frimition[1].SpeedPID.P = 10;
	frimition[1].SpeedPID.I = 0;
	frimition[1].SpeedPID.D = 0;
	frimition[1].SpeedPID.IMax = 700;
	frimition[1].SpeedPID.OutMax = 10000;
}

// void pluck_check(void) // ��������
// {
// 	if (pluck_stop == 0 && pluck.PositionPID.SetPoint - pluck.Angle_zeroCheck >= back_distance) // ������5Ȧ��Ϊ��ת
// 	{
// 		pluck_stop_position = pluck.Angle_zeroCheck;
// 		pluck.PositionPID.SetPoint = pluck.Angle_zeroCheck - back_decrease;
// 		if (droneState.ControlMode == 1) // ��������
// 		{
// 			pluck_stop = 1;
// 		}
// 	}
// 	else if (pluck_stop == 1 && droneState.ControlMode == cont_shoot && pluck_stop_position - pluck.Angle_zeroCheck <= back_distance)
// 	{

// 		pluck_stop = 0; // �������ָ̻�
// 	}
// }

float imu_filter(float imu_in)
{
	new_value = imu_in;
	if ((new_value - old_value > A) || (old_value - new_value > A))
	{
		return old_value;
	}
	return new_value;
}

static void Plane_ID_Shoot_Config(void)
{
#if Plane_ID == 1
	FrictionWheel_speed_stable1 = -8000;
	FrictionWheel_speed_stable2 = -8000;

	//plunk_inc_speed = 10000; // ����ת��
#elif Plane_ID == 2
	FrictionWheel_speed_stable1 = -10000; // 15000;// frictionwheel speed
	FrictionWheel_speed_stable2 = -10000;
	plunk_inc_speed = 15000; // ����ת��
#endif
}
/* end */
