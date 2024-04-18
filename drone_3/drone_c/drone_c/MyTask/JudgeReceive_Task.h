#ifndef JUDGE_RECEIVE_TASK_H
#define JUDGE_RECEIVE_TASK_H

#include "stdint.h"


//referee CommandCode enum
typedef enum{
	GAME_STATUS = 0x0001,
	GAME_ROBOT_HP = 0x0003,
	EVENT_DATA = 0x0101,
	DART_REMAIN_TIME = 0x0105,
	GAME_ROBOT_STATE = 0x0201,
	POWER_HEAT_DATA = 0x0202,
	GAME_ROBOT_POS = 0X203,
	BUFF_MUSK = 0X0204,
	ROBOT_HURT = 0x0206,
	SHOOT_DATA = 0X0207,
	BULLET_REMAIN = 0X0208,
	ROBOT_COMMAND = 0x0303
}COMMAND_CODE_ENUM;
typedef struct Test_Judge_Data 
{
	uint16_t Last_time;
	float Receive_Fre;
}Test_Judge_Data;
typedef struct Test_Judge_Receive_Fre_t
{
	Test_Judge_Data GAME_STATUS;
	Test_Judge_Data GAME_ROBOT_HP;
	Test_Judge_Data EVENT_DATA;
	Test_Judge_Data DART_REMAIN_TIME;
	Test_Judge_Data GAME_ROBOT_STATE;
	Test_Judge_Data POWER_HEAT_DATA;
	Test_Judge_Data GAME_ROBOT_POS;
	Test_Judge_Data ROBOT_HURT;
	Test_Judge_Data SHOOT_DATA;
	Test_Judge_Data BULLET_REMAIN;
	Test_Judge_Data ROBOT_COMMAND;
}Test_Judge_Receive_Fre_t;
#pragma pack(push, 1)
typedef struct // 帧头结构体
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t; 

typedef struct // 0x0001 扩展比赛状态数据
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

//typedef struct // 0x0002 比赛结果信息
//{
//	uint8_t winner;
//} ext_game_result_t;

typedef struct // 0x0003 扩展比赛机器人血量数据
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef struct // 0x0101 扩展事件数据
{
	//uint32_t event_type;
	uint8_t self_supply_status : 3;
	uint8_t self_Buff_status : 3;
	uint8_t self_highland_status : 6;
	uint8_t self_BaseShield : 7;
	uint16_t last_dart_time :9;
	uint8_t dart_target : 2;
	uint8_t gain_point_statuss : 2;
	//uint8_t _ : 3;
} ext_event_data_t;

typedef struct // 0x0102 补给站动作识别
{
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//typedef struct // 0x0103 补给站预定弹丸信息
//{
//	uint8_t supply_projectile_id;
//	uint8_t supply_robot_id;
//	uint8_t supply_num;
//} ext_supply_projectile_booking_t;

typedef struct // 0x0104 扩展裁判系统警告信息
{
	uint8_t level;
	uint8_t foul_robot_id;
	uint8_t count;
} ext_referee_warning_t;

typedef struct // 0x0105 飞镖剩余时间信息
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 0x020X --------------------------------------------------------------------*/
typedef struct // 0x0201 扩展比赛机器人状态
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef struct // 0x0202 实时功率和热量数据
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef struct // 0x0203 机器人位置信息
{
	float x;
	float y;
	float yaw;
} ext_game_robot_pos_t;

typedef struct // 0x0204 增益信息数据
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
} ext_buff_musk_t;

typedef struct // 0x205 空中支援数据
{ 
  uint8_t airforce_status; 
  uint8_t time_remain; 
}air_support_data_t; 

typedef struct // 0x0206 机器人受伤数据
{
	uint8_t armor_type : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef struct // 0x0207 实时射击数据
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

typedef struct // 0x0208 子弹剩余数量数据
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

//typedef struct // 0x0209 RFID状态信息
//{
//	uint32_t rfid_status;
//} ext_rfid_status_t;

//typedef struct // 0x020A 飞镖客户端命令信息
//{
//	uint8_t dart_launch_opening_status;
//	uint8_t dart_attack_target;
//	uint16_t target_change_time;
//	uint16_t operate_launch_cmd_time;
//} ext_dart_client_cmd_t;

/* 0x030X --------------------------------------------------------------------*/
//typedef struct // 0x0301 用户互动数据通信 头结构体
//{
//	uint16_t data_cmd_id;
//	uint16_t sender_ID;
//	uint16_t receiver_ID;
//} ext_student_interactive_header_data_t;

typedef struct //0x20B  地面机器人位置结构体
{ 
  float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float standard_5_x;  
  float standard_5_y; 
}ground_robot_position_t; 

typedef struct //0x20D
{
	uint16_t Sentry_convert_bullet : 11;
	uint8_t Sentry_convert_bullet_sum :4;
	uint8_t Sentry_convert_blood_sum : 4;
	uint16_t _ : 13;
}sentry_info_t;

typedef struct
{
	uint8_t radar_double_hurt_chance : 2;
	uint8_t radar_if_double_hurt : 1;
	uint8_t _ : 5;
}radar_info_t;

typedef struct // 0x0301  机器人交互数据结构体
{
	uint16_t data_cmd_id; 
	uint16_t sender_id; 
	uint16_t receiver_id; 
	uint8_t *data;
} robot_interactive_data_t;

typedef struct // 0x0303 机器人命令结构体
{
	float target_position_x;
	float target_position_y;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
	uint8_t cmd_source;
} ext_robot_command_t;

typedef struct // 0x0305 机器人地图命令结构体
{
	uint16_t target_robot_ID;
	float target_position_x;
	float target_position_y;
} ext_client_map_command_t;

//typedef struct //0x307 地图数据结构体
//{
//	uint8_t intention; 
//	uint16_t start_position_x; 
//	uint16_t start_position_y; 
//	int8_t delta_x[49]; 
//	int8_t delta_y[49]; 
//	uint16_t sender_id; 
//}map_data_t; 

typedef struct{
	uint8_t is_game_start : 1;
	uint8_t Heat_update : 1;
	uint8_t Robot_Red_Blue : 1; //1 -> red ; 0 -> blue
	uint8_t Enemy_Sentry_shootable : 1; //敌方哨兵是否可射击
	uint16_t self_outpost : 11;
	uint8_t Sentry_HomeReturned_flag : 1;
	uint16_t shooter1_heat;
	uint16_t bullet_remaining_num_17mm; //0x208
	uint16_t stage_remain_time; //0x0001
}JudgeData_ForSend1_t;

typedef struct{
	uint16_t x; //x坐标(锟斤拷float 映锟戒到 uint16_t : float*100 -> uint16_t)
	uint16_t y;
	uint8_t commd_keyboard; //命令键盘
	uint8_t Base_Shield;
	uint16_t Self_blood;
}JudgeData_ForSend2_t;

#pragma pack(pop)

typedef struct Referee_t
{
	/* 协议头结构体 */
	frame_header_struct_t Referee_Receive_Header;

	/* 0x000X */
	ext_game_status_t Game_Status; //0x0001
	ext_game_robot_HP_t Game_Robot_HP; //0x0003

	/* 0x010X */
	ext_event_data_t Event_Data; //0x0101

	ext_dart_remaining_time_t Dart_Remaining_Time; //0x0105

	/* 0x020X */
	ext_game_robot_state_t Game_Robot_State; //0x0201
	ext_power_heat_data_t Power_Heat_Data; //0x0202
	ext_game_robot_pos_t Game_Robot_Pos; //0x0203
	ext_buff_musk_t Buff_Musk; //0x0204
	ext_robot_hurt_t Robot_Hurt; //0x0206
	ext_shoot_data_t Shoot_Data; //0x0207
	ext_bullet_remaining_t Bullet_Remaining; //0x0208

	/* 0x030X */
	ext_robot_command_t Robot_Command; //0x0303

}Referee_t;

extern Referee_t Referee_All_Data;

void JudgeReceive_task(void);
#endif 
