#ifndef _ACTIONCONTROLTASK_H
#define _ACTIONCONTROLTASK_H

#include "main.h"
/*--------------------------模式类型----------------------------*/


typedef enum
{
    MouseKeyMode=0x001,
    RemoteMode  =0x002,
    CeaseMode   =0x000
} CtrMode_typedef;

typedef enum
{
    Chassis_Act_Mode			=	0x001,
    Chassis_SelfProtect_Mode  =	0x002,
    Chassis_CeaseMode   	=	0x000,
    Chassis_Single_Mode		= 0x005,
	  Chassis_Sniper_Mode   = 0x004,
	  Chassis_Solo_Mode     = 0x003
	  
} ChassisMode_typedef;

typedef enum
{
    Gimbal_Act_Mode			=	0x001,
    Gimbal_Armor_Mode  	=	0x002,
    Gimbal_CeaseMode   	=	0x000,
    Gimbal_TurnHeadMode = 0x003,
    Gimbal_SingleMode		= 0x004,
	  Gimbal_Lob_Mode     = 0x005
} GimbalMode_typedef;

typedef enum
{
    Shoot_Act_Mode			=	0x001,
    Shoot_CeaseMode   	=	0x000
} ShootMode_typedef;


//错误消息集合体，包括掉线，内存溢出，数据错误等，在调试时提供帮助

typedef struct
{
    short pre_key_z;
    short z_rise_flag;
    short pre_key_x;
    short x_rise_flag;
    short pre_key_c;
    short c_rise_flag;
    short pre_key_v;
    short v_rise_flag;
	  short pre_key_g;
    short g_rise_flag;
    short pre_key_b;
    short b_rise_flag;
    short pre_key_f;
    short f_rise_flag;
    short pre_key_r;
    short r_rise_flag;	
    short pre_mouse_l;
    short mouse_l_rise_flag;

    short pre_mouse_r;
    short mouse_r_rise_flag;

} Key_typedef;

typedef struct
{
    ChassisMode_typedef 	ChassisMode;
    GimbalMode_typedef		GimbalMode;
    ShootMode_typedef			ShootMode;
    CtrMode_typedef		CtrMode;
} HeroStatus_typedef;
void GraphicsUpdate(void);

void SetInputMode(RC_Ctl_t* RC_Ctl);
void Powerdown_Process(void);
void Remote_Process(RC_Ctl_t* RC_Ctl);
void Mouse_Key_Process(RC_Ctl_t* RC_Ctl);
void Change_Sensitivity(void);
void ActionControl_task(void const *pvParameters);
#endif
