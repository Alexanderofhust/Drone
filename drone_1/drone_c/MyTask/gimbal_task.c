/* include */

#include "main.h"
#include "gimbal_task.h"

/* declare */
/*----------------------------------�ⲿ����---------------------------*/


extern struct IMU 	 IMUReceive;
extern INS_t INS;

extern float new_value; 
extern float old_value; 
//extern short UI_Change_Mode;

//extern unsigned char SendToPC_Buff[USART4_PC_DMA_send_buffersize];

/*----------------------------------�ڲ�����---------------------------*/uint32_t gimbalinitTime;
float GimbalYawPos, GimbalPitchPos;

short Gimbal_Init = 1; 

float yaw_RC_sensitivity 									=	0.0012f;		//ң��ģʽyaw������_imu����
float pitch_RC_sensitivity									=	0.0006f;	//ң��ģʽpitch������ imu����

float yaw_RC_sensitivity_motor 								=	0.016f;		//ң��ģʽyaw������_�������
float pitch_RC_sensitivity_motor							=	0.008f;		//ң��ģʽpitch������ �������

float yaw_PC_Mouse_sensitivity								=	0.008f;		//����ģʽyaw��������� imu����
float pitch_PC_Mouse_sensitivity 							= 0.005f;		//����ģʽpitch��������� imu����

float yaw_PC_Mouse_sensitivity_motor						=	0.03f;		//����ģʽyaw��������� �������
float pitch_PC_Mouse_sensitivity_motor 						= 0.04f;		//����ģʽpitch��������� �������

float yaw_PC_Keyboard_sensitivity 							= 0.80f;		//����ģʽyaw���������� imu����
float pitch_PC_Keyboard_sensitivity 						= 0.25f;		//����ģʽpitch���������� imu����

float yaw_PC_Keyboard_sensitivity_motor 					= 0.10f;		//����ģʽyaw���������� �������
float pitch_PC_Keyboard_sensitivity_motor 					= 0.15f;		//����ģʽpitch���������� �������

int16_t YawCurrent, PitchCurrent;

static void imugimbal_config(void);
static void Plane_ID_Init(void);

static void Pitch_Motor_Test_G(float d_Pitch,uint16_t delay_time);

SawToothWave wave;
vector pitchfilter;
vector yawfilter;

#define GIMBALTEST   0			//���Բ�������
#define FREQTEST 0

#define YAW_IMULIMIT_LEFT  -135
#define YAW_IMULIMIT_RIGHT  135//6700

#define PITCH_IMULIMIT_HIGH  2
#define PITCH_IMULIMIT_LOW -40//-40

#define PITCH_MOTORLIMIT_HIGH	3	
#define PITCH_MOTORLIMIT_LOW	3	
	
/**********************************************************************************************************
*�� �� ��: Data_choose(void)
*����˵��: ���׵�ͨ�˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void SecondOrderFilter(vector *v) 
{	// ���潫vector��ֵ��v
	v->yout = v->k1*v->xin + v->k2*v->x1+v->k3*v->x2+ v->k4*v->y1 + v->k5*v->y2;            
	v->x2 = v->x1;    
	v->x1 = v->xin;                     
	v->y2 = v->y1;    
	v->y1 = v->yout; 
}
#if FREQTEST
extern float num;//������
extern float ARR;//��ʼƵ��Ϊ1hz
extern float freq;//Ƶ��
extern float circle;//�ض�Ƶ����ѭ�������趨
extern TIM_HandleTypeDef htim3;
void stop_TIM3(void)
{
	HAL_TIM_Base_Stop_IT(&htim3);
	num = 0;//������
	ARR = 1000;//��ʼƵ��Ϊ1hz
	freq = 1;//Ƶ��
	circle = 0;//�ض�Ƶ����ѭ�������趨
}
#endif


/**********************************************************************************************************
�������ƣ�IMU_Control_Limit()
����������float min,float max, struct GM6020 *Motor, float *set, float imuAngle
������;������������������ϵ����ˢ�µĻ��õ�������Ƶ���趨ֵset�ķ�Χ
**********************************************************************************************************/
void IMU_Control_Limit(float left,float right, struct GM6020 motor)
{
	
	static int flag = 1;
	static float motor_mid,imu_mid,left_limit,right_limit,Angle_now;
	static float left_imulimit,right_imulimit;
	
	if(flag)
	{
		motor_mid = motor.Angle_zeroCheck;
		imu_mid = motor.imuAngle;
		flag = 0;
	}
	
	left_limit = motor_mid + left*8192/360;
	right_limit = motor_mid + right*8192/360;
	
	
	left_imulimit = imu_mid + left;
	right_imulimit = imu_mid + right;
	
	
	if(motor.Angle_zeroCheck > right_limit && GimbalYawPos > right_imulimit)
	{
		GimbalYawPos = right_imulimit;
	}
	else if(motor.Angle_zeroCheck < left_limit && GimbalYawPos < left_imulimit)
	{
		GimbalYawPos = left_imulimit;
	}
}


/**********************************************************************************************************
*�� �� ��: Data_choose(void)
*����˵��: ������̨�ǵ���������������Ƿ���ѡ��pid����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

float testpitch;
float testyaw;
float a = 0.95;

short lastState;

void Data_choose(void)
{	

	static short mode = 1;
	static float lastyaw= 0 ;
	
	
	pitchfilter.xin = INS.PitchSpeed;
	yawfilter.xin = INS.YawSpeed;
	
	SecondOrderFilter(&pitchfilter);
	SecondOrderFilter(&yawfilter);
	yaw.imuAngle = -INS.YawTotalAngle;
	yaw.imuSpeed = yawfilter.yout;;
	testyaw = INS.YawSpeed;
	
	pitch.imuAngle = INS.Pitch;
	if(mode == 1)
		pitch.imuSpeed = pitchfilter.yout;
	if(mode == 2)
		pitch.imuSpeed = INS.PitchSpeed;
	if(mode == 3)
		pitch.imuSpeed = a*INS.PitchSpeed+(1-a)*testpitch;
	
	testpitch = INS.PitchSpeed;
	
	lastyaw = INS.YawTotalAngle;

	if(droneState.ControlMode != Lost)
	{
		if(droneState.MotorMode == Imu)
		{
			pitch.Speed = pitch.imuSpeed;
			pitch.Angle = pitch.imuAngle;
			
			yaw.Speed = yaw.imuSpeed;
			yaw.Angle = yaw.imuAngle;
			
			if(lastState == Motor ||  lastState == Lost)
				Gimbal_Init = 1;
			lastState = Imu;
		}
		else if(droneState.MotorMode == Motor ||  lastState == Lost)
		{
			pitch.Speed = pitch.MotorSpeed;
			pitch.Angle = pitch.Angle_zeroCheck;
			
			yaw.Speed = yaw.MotorSpeed;
			yaw.Angle = yaw.Angle_zeroCheck;
			
			
			if(lastState == Imu)
				Gimbal_Init = 1;
			
			lastState = Motor;
		
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Auto_gimbal()
*����˵��: ������̨�����趨
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void AutoGimbal(void)
{
	#if FREQTEST!=1
	if(Gimbal_Init)
	{
		GimbalYawPos = yaw.Angle;			
		GimbalPitchPos = pitch.Angle;
		Gimbal_Init = 0;
		pid_reset(&pitch.PositionPID[droneState.MotorMode]);
		pid_reset(&yaw.PositionPID[droneState.MotorMode]);

	}
		
	/**************�����������ֵΪ0,��setpoint��Ϊimu��ǰֵ   */
	if(PC_Recv.yaw!=0)
		GimbalYawPos = PC_Recv.yaw;			
	else 
		GimbalYawPos = yaw.Angle;		
	
	if(PC_Recv.pitch!=0)
		GimbalPitchPos = PC_Recv.pitch;
	else
		GimbalPitchPos = pitch.Angle;
	
	//IMU_Control_Limit(yaw_left_limit,yaw_right_limit,yaw,&set_yaw,imu.yawAngle);//����ǿ�����������λ
	//limit_max_min(&set_pitch,pitch_high_limit_imu,pitch_low_limit_imu); 
	
	yaw.PositionPID[droneState.MotorMode].SetPoint = GimbalYawPos;
	pitch.PositionPID[droneState.MotorMode].SetPoint = GimbalPitchPos;
	#else
	if(Gimbal_Init)
	{
		GimbalYawPos = 0;			
		GimbalPitchPos = pitch.Angle;
		Gimbal_Init = 0;
		pid_reset(&pitch.PositionPID[droneState.MotorMode]);
		pid_reset(&yaw.PositionPID[droneState.MotorMode]);
		SawWaveRun();
	}
	
	
	yaw.PositionPID[droneState.MotorMode].SetPoint = wave.out;
	#endif

}

/**********************************************************************************************************
*�� �� ��: Pitch_Motor_Test_G()
*����˵��: �������������������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float Balance_Pitch_Current,Balance_Pitch,Balance_Pitch_Current_Float;
static void Pitch_Motor_Test_G(float d_Pitch,uint16_t delay_time)
{
	static float Pitch_Low = -43;
	static float Pitch_Max = -10;
	static uint16_t delay_num = 0;
	static uint8_t Success_Flag = 0;
	static uint8_t Init_Flag = 0;
	delay_num++;
	if(!Init_Flag)
	{
		if(delay_num > 75)
		{
			delay_num = 0;
			Init_Flag = 1;
			Success_Flag = 0;
		}
		GimbalPitchPos = Pitch_Low;
	}else{
		if(delay_num > delay_time && !Success_Flag)
		{
			GimbalPitchPos += d_Pitch;
			delay_num = 0;
			Balance_Pitch_Current = pitch.torque;
			Balance_Pitch = pitch.imuAngle;
			Balance_Pitch_Current_Float = (float)(Balance_Pitch_Current);
		}
	}
	if(pitch.imuAngle > Pitch_Max - 0.4 && pitch.imuAngle < Pitch_Max + 0.4)
	{
		Success_Flag = 1;
	}
	pitch.PositionPID[0].SetPoint = GimbalPitchPos;
	
	
	
}



/**********************************************************************************************************
*�� �� ��: Calc_Pitch_Balance_Current()
*����˵��: ������������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float Grativity = 0;
float p1 = -0.058;
float p2 = -5.70;
float p3 = -196.8;
float p4 = -2700;
float p5 = -7515;

float Calc_Pitch_Balance_Current(float Pitch_Groy)
{
	return (float)(p1*Pitch_Groy*Pitch_Groy*Pitch_Groy*Pitch_Groy + p2*Pitch_Groy*Pitch_Groy*Pitch_Groy + p3*Pitch_Groy*Pitch_Groy + p4*Pitch_Groy+p5);
}



/****
�������ƣ�RC_gimbal()
����������void
������;��ң������̨�����趨
****/
void RcGimbal(void)
{
	//����л���̨���ߵ������ģʽ��������ȷ����ʼ�̶�����
	if(Gimbal_Init)
	{
		GimbalYawPos = yaw.Angle;			
		GimbalPitchPos = pitch.Angle;
		Gimbal_Init = 0;
		pid_reset(&pitch.PositionPID[droneState.MotorMode]);
		pid_reset(&yaw.PositionPID[droneState.MotorMode]);
#if FREQTEST
//		stop_TIM3();
		GimbalYawPos = 0;
		SawWaveStop(&wave,1);
		
#endif
		
	}

	
#if GIMBALTEST
	Pitch_Motor_Test_G(0.5,50);
#else
	if(droneState.MotorMode == Imu)
	{
		GimbalYawPos += (((float)RC_Ctl.rc.ch0 - 1024) * yaw_RC_sensitivity);//0.008f
		GimbalPitchPos +=  ((float)RC_Ctl.rc.ch1 - 1024) * pitch_RC_sensitivity;		//0.015f
		
		//�����������
//		Grativity = Calc_Pitch_Balance_Current(pitch.imuAngle);
		Grativity = 0;
	}
	else if(droneState.MotorMode == Motor)
	{
		GimbalYawPos += (((float)RC_Ctl.rc.ch0 - 1024) * yaw_RC_sensitivity_motor);//0.008f
		GimbalPitchPos +=  ((float)RC_Ctl.rc.ch1 - 1024) * pitch_RC_sensitivity_motor;		//0.015f
	}
	else
		Grativity = 0;
	
	GimbalPitchPos -= (float)RC_Ctl.mouse.y * pitch_PC_Mouse_sensitivity;
	GimbalYawPos += (float)RC_Ctl.mouse.x * yaw_PC_Mouse_sensitivity;
	
	
	//�޷�
	GimbalYawPos = LIMIT_MAX_MIN(GimbalYawPos,YAW_IMULIMIT_RIGHT,YAW_IMULIMIT_LEFT); 
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,PITCH_IMULIMIT_HIGH,PITCH_IMULIMIT_LOW); 
		
	yaw.PositionPID[droneState.MotorMode].SetPoint = GimbalYawPos;
	pitch.PositionPID[droneState.MotorMode].SetPoint = GimbalPitchPos;
	
	//pitch.ffout = FeedForward_Cal(&pitch.AngelFF ,IMUReceive.pitchAngle);//pitchǰ��
#endif
	
}

void LostGimbal(void)//����ģʽ
{
	
	#if FREQTEST
		//stop_TIM3();
	SawWaveStop(&wave,1);
	#endif
	RC_Ctl.rc.ch0=1024;
	RC_Ctl.rc.ch1=1024;
	RC_Ctl.rc.ch2=1024;
	RC_Ctl.rc.ch3=1024;

	YawCurrent = 0;
	PitchCurrent = 0;

	lastState = Lost;
}




short test = 0;
void Motor_control(void)
{
	
	if(droneState.ControlMode != Lost)
	{
		yaw.SpeedPID[droneState.MotorMode].ActualValue = yaw.Speed;
		yaw.PositionPID[droneState.MotorMode].ActualValue = yaw.Angle;
		YawCurrent = PID_Concatenation_cal(&yaw.PositionPID[droneState.MotorMode],&yaw.SpeedPID[droneState.MotorMode]);
	
		pitch.SpeedPID[droneState.MotorMode].ActualValue = pitch.Speed;
		pitch.PositionPID[droneState.MotorMode].ActualValue = pitch.Angle;
		
		
		PitchCurrent = PID_Concatenation_cal(&pitch.PositionPID[droneState.MotorMode],&pitch.SpeedPID[droneState.MotorMode])
			+FeedForward_Cal(&pitch.AngleFF,pitch.Angle) + Grativity;
	
		
		YawCurrent=LIMIT_MAX_MIN(YawCurrent,30000,-30000);
		PitchCurrent=LIMIT_MAX_MIN(PitchCurrent,16000,-16000);
	}
	
	if(test == 0)
		GM6020_Send_can1(YawCurrent,PitchCurrent);
	else
		GM6020_Send_can1(0,0);

}




/*
 *    			S2							    					S1    
 *  1 --> ������̨�����(���)    1 --> ����ģʽ	
 * 	                             
 *  3 --> ������̨�����(IMU)     3 --> ����ģʽ	
 *	                     								
 
 *  2 --> ����ģʽ     						2 --> ң��ģʽ
 *					
 * (���ظ���: S1 turn from rc mode to key mode and turn back in 1.5 seconds) 
 */
 //IMU���ұ�� YAW���ұ��
 //IMU�ϵ��´�С,�����¸�,PITCH�ϵ��±�С

/* IMU��̨�������� */
void GimbalTask(void const *pvParameters)
{
	/* IMU��̨�������� */
	imugimbal_config();
	Plane_ID_Init();

	yaw.zero.Circle=0;
	pitch.zero.Circle=0;

	SawToothInit(&wave,1,30);
	
	for(;;)
	{		
		Data_choose();
		/* gimbal running */		
		//if((droneState.drone.droneState&0x55) == 0 && droneState.ControlMode != Lost)
			switch(droneState.ControlMode)
			{
				case Rc:
				case Rc_Shoot:
					RcGimbal();
					break;
				case Auto:
					AutoGimbal();
					break;
				case Lost:
					LostGimbal();
					break;
				default:
					LostGimbal();
					break;
			}
//		else
//		{	//����ģʽ
//			LostGimbal();
//		}
		
			
		taskENTER_CRITICAL();
		Motor_control();
		taskEXIT_CRITICAL();
		osDelay(5);
	}
}




/* imu gimbal config */


void imugimbal_config(void)			//imu��̨��ʼ������
{
#if Plane_ID == 1
	//FeedForwardInit();
	
	/* yaw���ʼ�� */
	yaw.id = 0;
	yaw.receiveId = 0x205;
	ZeroCheck_Init_Interface(&yaw.zero,8191,yaw.MotorAngle);
	
	//pitch���׵�ͨ�˲���ʼ��
	yawfilter.k1 = 0.5;
	yawfilter.k2 = 0.15;
	yawfilter.k3 = 0.1;
	yawfilter.k4 = 0.2;
	yawfilter.k5 = 0.05;
	//������PID
	yaw.PositionPID[0].P = 19.5f;
	yaw.PositionPID[0].I = 0.005f;	
	yaw.PositionPID[0].D = 0.0f;
	yaw.PositionPID[0].IMax = 100.0f;
	yaw.PositionPID[0].OutMax = 30000.0f;


	yaw.SpeedPID[0].P = 190.0f;
	yaw.SpeedPID[0].I = 5.0f;	
	yaw.SpeedPID[0].D = 0.0f;
	yaw.SpeedPID[0].IMax = 400;
	yaw.SpeedPID[0].OutMax = 30000;
	
	yaw.AngleFF.param[0] = 0;
	yaw.AngleFF.param[1] = 0;
	yaw.AngleFF.param[2] = 0;
	
	//����Ƕ�PID
	yaw.PositionPID[1].P = 0.4;
	yaw.PositionPID[1].I = 0.003;	
	yaw.PositionPID[1].D = 0.38;
	yaw.PositionPID[1].IMax = 100;
	yaw.PositionPID[1].OutMax = 30000;
	
	yaw.SpeedPID[1].P = 450;
	yaw.SpeedPID[1].I = 40;	
	yaw.SpeedPID[1].D = 0.01;
	yaw.SpeedPID[1].IMax = 100;
	yaw.SpeedPID[1].OutMax = 30000;
	
	/* pitch���ʼ�� */
	pitch.id = 0;
	pitch.receiveId = 0x206;
	ZeroCheck_Init_Interface(&pitch.zero,8191,pitch.MotorAngle);	
	
	//pitch���׵�ͨ�˲���ʼ��
	pitchfilter.k1 = 0.5;
	pitchfilter.k2 = 0.15;
	pitchfilter.k3 = 0.1;
	pitchfilter.k4 = 0.2;
	pitchfilter.k5 = 0.05;
	
	//������PID
	pitch.PositionPID[0].P = 32.00f;//20
	pitch.PositionPID[0].I = 0.2f;	
	pitch.PositionPID[0].D = 0.0f;
	pitch.PositionPID[0].IMax = 1000;
	pitch.PositionPID[0].OutMax = 16000;
	
	pitch.SpeedPID[0].P = 110.0f;
	pitch.SpeedPID[0].I = 0.5f;	
	pitch.SpeedPID[0].D = 0;
	pitch.SpeedPID[0].IMax = 1000;
	pitch.SpeedPID[0].OutMax = 16000;
	
	pitch.AngleFF.param[0] = 0;
	pitch.AngleFF.param[1] = 0;
	pitch.AngleFF.param[2] = 0;
	
	//�����PID
	pitch.PositionPID[1].P = 20;
	pitch.PositionPID[1].I = 0;	
	pitch.PositionPID[1].D = 0;
	pitch.PositionPID[1].IMax = 200;
	pitch.PositionPID[1].OutMax = 16000;

	pitch.SpeedPID[1].P = 15;
	pitch.SpeedPID[1].I = 0.4;	
	pitch.SpeedPID[1].D = 16;
	pitch.SpeedPID[1].IMax = 400;
	pitch.SpeedPID[1].OutMax = 16000;

#elif Plane_ID ==2
	FeedForwardInit();
	/* yaw���ʼ�� */
	yaw.id = 0;
	yaw.receiveId = 0x207;
	ZeroCheck_Init_Interface(&yaw.zero,8191,yaw.receiveAngle);
	yaw.mode = imuYaw;	
	//								PID�ṹ��	 mode ����޷�	�����޷�	 P				 I			D		
	//��ͨPID---RC	
	PID_struct_init(&yaw.speedPID,		1,	30000.0f,	100.0f,	45.0f,0.5f, -0.0f);	//-1700.0f,-0.00f, -2500.0f   75.0f 0 0
	PID_struct_init(&yaw.positionPID,	1,	30000.0f,	100.0f,	2.5f,		0.0f,	 2.0f	 );	//1.405f,		0.0051f,	 .9f		2.5F 0.005 1
	//���������---RC
	PID_struct_init(&yaw.Motor_SpeedPID ,		1,	30000.0f,	100.0f,	-350.0f, -0.01f, -0.0f);	
	PID_struct_init(&yaw.Motor_PositionPID,	1,	30000.0f,	100.0f,	0.5f,		0.0001f,	 0.0f	 );	
	
	//�������  speed 1,	30000.0f,	100.0f,	45.0f, 0.0001f, -0.0f
	//   				position 1,	30000.0f,	100.0f,	1.5f,		0.0005f,	 1.0f	 
	//ģ��PID---RC
	Fuzzy_PID_struct_init(&yaw.speedPID_fuzzy, 30000.0f, 100.0f,15.0f,0.3f,-0.3f,0.0f,0.0f,0.0f,0.4f,0.2f,0.0f,0.5f,5.0f,0.5f,0.15f,0.1f);
	Fuzzy_PID_struct_init(&yaw.positionPID_fuzzy, 30000.0f, 100.0f,8.0f,0.0f,-0.0f,0.0f,0.0f,0.0f,0.4f,0.2f,0.0f,0.5f,3.0f,0.5f,0.05f,0.1f);
	//����PID---Auto
	PID_struct_init(&yaw.AutospeedPID,		1,	30000.0f,	35.0f,	-4500.0f,	-10.0f, -600.0f);  //ͬRc
	PID_struct_init(&yaw.AutopositionPID,	1,	30000.0f,	35.0f,	 0.35f,		0.02f,	 0.0f);	//0.2 0.004 0.3
	
	/* pitch���ʼ�� */
	pitch.id = 0;
	pitch.receiveId = 0x206;
	ZeroCheck_Init_Interface(&pitch.zero,8191,pitch.receiveAngle);
	pitch.mode = imuPitch;	
	
	//									PID�ṹ��	  mode ����޷�	 �����޷�	 P			I			  D		
	//��ͨPID---RC	
	PID_struct_init(&pitch.speedPID,		1,	30000.0f,	100.0f,	700.0f,	 -20.0f, -0.0f);//6000f,	0.00f, 8000f   -46 0.001
	PID_struct_init(&pitch.positionPID,	1,	30000.0f,	200.0f,	0.2f,		0.0f,	0.0f	 );//0.15f,		0.00f,	0.15f	   0.1   0.0001
	//���������---RC
	PID_struct_init(&pitch.Motor_SpeedPID,		1,	30000.0f,	100.0f,	-50.0f,	 0.0f, 0.0f); //-50 0 0
	PID_struct_init(&pitch.Motor_PositionPID,	1,	30000.0f,	200.0f,	2.0f,		0.001f,	1.5f	 );//1.5 0.01 1.5
	//ģ��PID---RC
	Fuzzy_PID_struct_init(&pitch.speedPID_fuzzy, 30000.0f, 100.0f,15.0f,0.2f,-0.3f,0.0f,0.0f,0.0f,0.4f,0.2f,0.0f,0.5f,5.0f,0.5f,0.15f,0.1f);
	Fuzzy_PID_struct_init(&pitch.positionPID_fuzzy, 30000.0f, 100.0f,8.0f,0.0f,-0.0f,0.0f,0.0f,0.0f,0.4f,0.2f,0.0f,0.5f,3.0f,0.5f,0.05f,0.1f);
	//����PID---Auto
	PID_struct_init(&pitch.AutospeedPID,		1,	30000.0f,	20000.0f,	4100.0f,	3.0f, 4000.0f); //ͬRc
	PID_struct_init(&pitch.AutopositionPID,	1,	30000.0f,	100.0f,	 0.25f, 0.015f,	 0.0f	 );	//0.4 0.32
#endif
}

static void Plane_ID_Init(void )
{
#if Plane_ID == 1
//*************************************************		

		
	
#elif Plane_ID == 2
//*************************************************	
		yaw_left_limit  = 5200;
		yaw_right_limit =  8100;
    pitch_high_limit_imu  = 10; 
		pitch_low_limit_imu = -30;4
		pitch_high_limit_motor = 7500;
		pitch_low_limit_motor = 6500;
	
#endif

}
/* end */

