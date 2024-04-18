/* include */
#include "selfcheck_task.h"
#include "main.h"

struct state droneState;

/* declare */
extern struct IMU 	 IMUReceive;

static void selfCheck_update(void);
//static void led_update(void);
static uint16_t checkComponent(uint32_t now,uint32_t last,uint32_t delay);
static void device_lost_reset(void);

int IMU_dis =0;

/* 初始化时首先开机自检 然后300ms检查一次信号更新(运行自检) */
void SelfcheckTask(void const *pvParameters)
{
	static uint32_t initTime;
	
	initTime = xTaskGetTickCount();
	
	/* 红蓝LED常亮提示自检进行中 */
//	led_selfCheck();
	//IWDG_Config(64,1300);
	/* 自检300ms */
	while(xTaskGetTickCount()-initTime<1000)
	{
		/* 更新自检数据 */
		selfCheck_update();
		//IWDG_Feed();
		vTaskDelay(1);
		droneState.ControlMode = Lost;
		droneState.MotorMode = Imu;
		droneState.ShootMode = stop_shoot;
	}
	
	/* 开机自检正常 则复位历史掉线记录 */
	if((droneState.drone.droneState == 0)&&(SELFCHECK == 1))//自检开启
	{
		device_lost_reset();
	}
//	/* 所有设备在线 或 系统配置放弃开机自检 */
//	if((((droneState.drone.droneState)==0) && (SELFCHECK ==1))||(SELFCHECK == 0))//状态正常或屏蔽自检
//	{
//		/* 正常控制 创建云台和射击任务 */F
//		control_gimbal();
//		control_shoot();
//	}

	/* 运行自检任务主体 */
	for(;;)
	{
		/* 运行中自检频率100ms */
		selfCheck_update();
		/* LED状态更新 */
//		led_update();

		
//		/* 飞行中自检正常后恢复云台控制 */
//		/* (所有设备在线 或 上场状态) 且 当前未获得控制权 */ 
//		if(((droneState.drone.droneState)== 0)||(SELFCHECK == 0))//状态正常或屏蔽自检
//		{
//			/* 当前自检正常 恢复正常控制 */		
//			control_gimbal();
//			control_shoot();			
//		}
		
		/* 飞行中设备掉线放弃云台控制 */
		/* 正常控制中有设备掉线(飞行状态) */
//		if (droneState.drone.droneState != 0)
//		{
//				dis_control();					
//		}
		//IWDG_Feed();
		vTaskDelay(100);
	}
}

///* 创建云台控制任务 */
//static void control_gimbal(void)
//{
//	/* yaw pitch自检通过 */
//	if(((((droneState.drone.composeState.yaw_disconneced==0)||(droneState.drone.composeState.pitch_disconneced==0))))&& (dis_control_gimbal_flag == 1))
//	{
//		/* 创建IMU云台任务 */
//		xTaskCreate((TaskFunction_t)ImugimbalTask,      	
//								(const char *	)"Imu cotrol gimbal",     
//								(uint16_t			)IMU_GIMBAL_TASK_SIZE,    
//								(void *				)NULL,              
//								(UBaseType_t		)IMU_GIMBAL_TASK_PRIO,    
//								(TaskHandle_t *)&ImugimbalTask_Handler);
//		dis_control_gimbal_flag = 0;
//	}
//}

///* 创建射击控制任务 */
//static void control_shoot(void)
//{
//	/* 所有自检通过*/
//	if(((droneState.drone.droneState == 0)||(SELFCHECK == 0))&&( dis_control_shoot_flag == 1))
//	{
//		/* 创建射击任务 */
//		xTaskCreate((TaskFunction_t)ShootTask,      	
//							(const char *	)"shoot",     
//							(uint16_t			)SHOOT_TASK_SIZE,    
//							(void *				)NULL,              
//							(UBaseType_t		)SHOOT_TASK_PRIO,    
//							(TaskHandle_t *)&ShootTask_Handler);
//		dis_control_shoot_flag=0;
//	}
//}

/* 放弃控制权 */
//static uint8_t dis_control(void)
//{
//	/* 发射机构掉电前处理 */
//	if(shoot_ProTerminate())//?
//	{
//		/* 放弃控制权 */
//		if(dis_control_shoot_flag==0)
//		{
//			dis_control_shoot_flag=1;
//		  vTaskDelete(ShootTask);
//		}
//		if(dis_control_gimbal_flag==0)
//		{
//			dis_control_gimbal_flag=1;
//		  vTaskDelete(ImugimbalTask);
//	  }
//		return 1;
//	}
//}



/* 自检数据更新 */
static void selfCheck_update(void)
{
	/* 更新时间 */
	uint32_t nowTime = xTaskGetTickCount();
	/* 更新IMU状态 */
	droneState.drone.composeState.IMU_disconneced = checkComponent(nowTime,IMUReceive.updateTime,100);
	//droneState.drone.composeState.IMU_disconneced = 0;
	/* 更新YAW轴电机状态 */
	droneState.drone.composeState.yaw_disconneced = checkComponent(nowTime,yaw.updateTime,100);
	/* 更新PITCH轴电机状态 */
	droneState.drone.composeState.pitch_disconneced = checkComponent(nowTime,pitch.updateTime,100);
	/* 更新RC状态 */
	droneState.drone.composeState.RC_disconneced = checkComponent(nowTime,RC_Ctl.updateTime,500);
	/* 更新拨弹电机状态 */
	droneState.drone.composeState.Pluck_disconneced = checkComponent(nowTime,pluck.updateTime,100);
	/* 更新云台上电状态 */
//	if(RC_Ctl.rc.s1 == 2)
//		droneState.Gimbal_poweroff = 1;
//	else
//		droneState.Gimbal_poweroff = 0;
	/* 更新辅瞄开启状态 */
	
	/* 更新PC数据接收状态 */
	//droneState.PC_Received = checkComponent(nowTime,PC_Recv.updateTime,200);
	
	/* 历史掉线记录 */
	if(droneState.drone.composeState.IMU_disconneced == 1)
		IMUReceive.Ever_Lost = 1;
	if(droneState.drone.composeState.yaw_disconneced == 1)
		yaw.Ever_Lost = 1;
	if(droneState.drone.composeState.pitch_disconneced == 1)
		pitch.Ever_Lost = 1;
	if(droneState.drone.composeState.RC_disconneced == 1)
		RC_Ctl.Ever_Lost = 1;
	if(droneState.drone.composeState.Pluck_disconneced == 1)
		pluck.Ever_Lost = 1;
}

/* 模块自检 */
/* 自检超时 -> 返回1 */
/* 自检正常 -> 返回0 */
/**********************************************************************************************************
*函 数 名: checkComponent
*功能说明: 模块自检，can接收时更新时间
*形    参: uint32_t now,uint32_t last,uint32_t delay
*返 回 值: 无
**********************************************************************************************************/
static uint16_t checkComponent(uint32_t now,uint32_t last,uint32_t delay)
{
	if((now-last)>delay)//通信超时
	{
		return 1;//认为断连
	}
	else
	{		
		return 0;
	}
}

/* 掉电历史自检 */
static void device_lost_reset(void)
{
//	IMUReceive.Ever_Lost = 1;
	yaw.Ever_Lost = 1;
	pitch.Ever_Lost = 1;
	RC_Ctl.Ever_Lost = 1;
	pluck.Ever_Lost = 1;
}


/* end */

