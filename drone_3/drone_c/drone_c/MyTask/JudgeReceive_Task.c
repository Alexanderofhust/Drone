#include "JudgeReceive_Task.h"
#include "main.h"
#include "crc.h"

Referee_t Referee_All_Data;
JudgeData_ForSend1_t JudgeData_ForSend1;
JudgeData_ForSend2_t JudgeData_ForSend2;
uint8_t If_Game_Start = 0;
float Last_chassisPower=0;

extern TaskHandle_t JudgeReceiveTask_Handler; //??????
extern uint8_t JudgeReveice_Flag;
extern unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];

static unsigned char JudgeDataFIFO[2*JudgeBufBiggestSize]; 

float crc_sum=0;
float crc_err_sum = 0;
float err_ratio;

float crc_sum_16=0;
float crc_err_sum_16 = 0;
float err_ratio_16;

uint16_t Pack_Point = 0;

static uint8_t JudgeDataSolve_Init_Flag = 0;

uint16_t pass_head_flag = 0;
Test_Judge_Receive_Fre_t  Test_Judge_Receive_Fre;
static void JudgeDataSolve(unsigned char ReceiveBuffer[],uint16_t Receive_Len)
{
	static uint16_t cmd_id;

	uint16_t DataLen = 0;
	memcpy(&JudgeDataFIFO[0],&ReceiveBuffer[0],Receive_Len);
	
	for(Pack_Point=0;Pack_Point<Receive_Len;Pack_Point++)
	{
		if(JudgeDataFIFO[Pack_Point]==0xA5) 
		{
			crc_sum = crc_sum+1;			
			if(Verify_CRC8_Check_Sum(&JudgeDataFIFO[Pack_Point],5)==1)
			{
				if(Pack_Point+DataLen+9 >= Receive_Len)
					break;
				crc_sum_16 +=1;
				cmd_id=(JudgeDataFIFO[Pack_Point+6])&0xff;
				cmd_id=(cmd_id<<8)|JudgeDataFIFO[Pack_Point+5];  
				DataLen=JudgeDataFIFO[Pack_Point+2]&0xff;
				DataLen=(DataLen<<8)|JudgeDataFIFO[Pack_Point+1];
				
				if(1 == (Verify_CRC16_Check_Sum(&JudgeDataFIFO[Pack_Point],DataLen+9))) 
				{
					switch(cmd_id) 
					{
						case GAME_ROBOT_STATE: //0x201 10hz
							memcpy(&Referee_All_Data.Game_Robot_State,&JudgeDataFIFO[Pack_Point+7],DataLen);
							JudgeData_ForSend2.Self_blood = Referee_All_Data.Game_Robot_State.remain_HP;
							if(Referee_All_Data.Game_Robot_State.robot_id < 10)
							{
								//F105.Sendmessage.RobotRed = 1;
								JudgeData_ForSend1.Robot_Red_Blue = 1;
							}
							else
							{
								//F105.Sendmessage.RobotRed = 0;
								JudgeData_ForSend1.Robot_Red_Blue = 0;
							}
//							Can2Send2(&JudgeData_ForSend2);
							Test_Judge_Receive_Fre.GAME_ROBOT_STATE.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.GAME_ROBOT_STATE.Last_time)); 
							break;
						
						case GAME_ROBOT_HP: //0x003 3hz
							memcpy(&Referee_All_Data.Game_Robot_HP,&JudgeDataFIFO[Pack_Point+7],DataLen);
							if(JudgeData_ForSend1.Robot_Red_Blue == 1)
							{
								JudgeData_ForSend1.Enemy_Sentry_shootable = (0 == Referee_All_Data.Game_Robot_HP.blue_outpost_HP);
							}
							else
							{
								JudgeData_ForSend1.Enemy_Sentry_shootable = (0 == Referee_All_Data.Game_Robot_HP.red_outpost_HP);
							}
							Test_Judge_Receive_Fre.GAME_ROBOT_HP.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.GAME_ROBOT_HP.Last_time)); 

							break;
							
						case POWER_HEAT_DATA: //0x202 50hz
							memcpy(&Referee_All_Data.Power_Heat_Data,&JudgeDataFIFO[Pack_Point+7],DataLen);
							//F105.Sendmessage.heat_update = 0x01;
							//F105.Sendmessage.shooterHeat17 = Referee_All_Data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;
							JudgeData_ForSend1.Heat_update = 0x01;
							JudgeData_ForSend1.shooter1_heat = Referee_All_Data.Power_Heat_Data.shooter_id1_17mm_cooling_heat;
//							Can2Send1(&JudgeData_ForSend1);
							JudgeData_ForSend1.Heat_update = 0x0;
							Test_Judge_Receive_Fre.POWER_HEAT_DATA.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.POWER_HEAT_DATA.Last_time)); 
							break;
						case GAME_ROBOT_POS: //0x203 10hz
							memcpy(&Referee_All_Data.Game_Robot_Pos,&JudgeDataFIFO[Pack_Point+7],DataLen);
							JudgeData_ForSend2.x = (uint16_t)(Referee_All_Data.Game_Robot_Pos.x * 100);
							JudgeData_ForSend2.y = (uint16_t)(Referee_All_Data.Game_Robot_Pos.y * 100);
							//Can2Send2(&JudgeData_ForSend2);
							Test_Judge_Receive_Fre.GAME_ROBOT_POS.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.GAME_ROBOT_POS.Last_time)); 
							break;
		//					case BUFF_MUSK:
		//						memcpy(&Referee_All_Data.Buff_Musk,&JudgeDataFIFO[Pack_Point+7],DataLen);
		//						//Can2Send2(JudgeDataFIFO[Pack_Point+7]);
		//						break;
		//					case ROBOT_HURT:
		//						memcpy(&Referee_All_Data.Robot_Hurt,&JudgeDataFIFO[Pack_Point+7],DataLen);
		//						break;
		//					case SHOOT_DATA:
		//						memcpy(&Referee_All_Data.Shoot_Data,&JudgeDataFIFO[Pack_Point+7],DataLen);
		//						break;
						case BULLET_REMAIN: //0x208 10hz 
							memcpy(&Referee_All_Data.Bullet_Remaining,&JudgeDataFIFO[Pack_Point+7],DataLen);
							JudgeData_ForSend1.bullet_remaining_num_17mm = Referee_All_Data.Bullet_Remaining.bullet_remaining_num_17mm;
							Test_Judge_Receive_Fre.BULLET_REMAIN.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.BULLET_REMAIN.Last_time)); 
							break;
						case GAME_STATUS: //0x001 3hz
							memcpy(&Referee_All_Data.Game_Status,&JudgeDataFIFO[Pack_Point+7],DataLen);
							If_Game_Start = (Referee_All_Data.Game_Status.game_progress >=0x04)?1:0;
							JudgeData_ForSend1.is_game_start = If_Game_Start;
							JudgeData_ForSend1.stage_remain_time = Referee_All_Data.Game_Status.stage_remain_time;
							Test_Judge_Receive_Fre.GAME_STATUS.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.GAME_STATUS.Last_time));
							break;
		//				case GAME_ROBOT_HP:
		//					 
		//					memcpy(&Referee_All_Data.Game_Robot_HP,&JudgeDataFIFO[Pack_Point+7],DataLen);
		//					
		//					if(If_Game_Start == 1 && Referee_All_Data.Game_Status.stage_remain_time >= 415)
		//					{
		//						if(F105.Sendmessage.RobotRed)
		//						{
		//							if(Referee_All_Data.Game_Robot_HP.blue_3_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 );
		//							if(Referee_All_Data.Game_Robot_HP.blue_4_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 <<1);
		//							if(Referee_All_Data.Game_Robot_HP.blue_5_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 <<2);
		//						}
		//						else
		//						{
		//							if(Referee_All_Data.Game_Robot_HP.red_3_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 );
		//							if(Referee_All_Data.Game_Robot_HP.red_4_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 <<1);
		//							if(Referee_All_Data.Game_Robot_HP.red_5_robot_HP == 300)
		//								F105.Sendmessage.which_balance = F105.Sendmessage.which_balance | (0x01 <<2);
		//						}
		//					}
		//					break;
						case EVENT_DATA: // 0x101 3hz
							memcpy(&Referee_All_Data.Event_Data,&JudgeDataFIFO[Pack_Point+7],DataLen);
							//JudgeData_ForSend1.self_outpost = Referee_All_Data.Event_Data.self_outpost;
							//JudgeData_ForSend1.Sentry_HomeReturned_flag = Referee_All_Data.Event_Data.Sentry_HomeReturned_flag;
							JudgeData_ForSend2.Base_Shield = Referee_All_Data.Event_Data.self_BaseShield;
							Test_Judge_Receive_Fre.EVENT_DATA.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.EVENT_DATA.Last_time));
							break;
						case DART_REMAIN_TIME: //0x105 3hz
							memcpy(&Referee_All_Data.Dart_Remaining_Time,&JudgeDataFIFO[Pack_Point+7],DataLen);
							Test_Judge_Receive_Fre.DART_REMAIN_TIME.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.DART_REMAIN_TIME.Last_time));
							break;
						case ROBOT_COMMAND: //0x303 at any time 
							memcpy(&Referee_All_Data.Robot_Command,&JudgeDataFIFO[Pack_Point+7],DataLen);
							JudgeData_ForSend2.commd_keyboard = Referee_All_Data.Robot_Command.commd_keyboard;
							//JudgeData_ForSend2.KeyBoard_Update = 0x1;
							//Can2Send2(&JudgeData_ForSend2);
							Test_Judge_Receive_Fre.ROBOT_COMMAND.Receive_Fre=1.0f/GetDeltaT(&(Test_Judge_Receive_Fre.ROBOT_COMMAND.Last_time));
							break;
						default:
							break;
					}
					Pack_Point += DataLen + 8;
				}else
				{
					crc_err_sum_16 += 1;
				}	
				
			}
			else
			{
				crc_err_sum = crc_err_sum + 1;
			}
			err_ratio = crc_err_sum / crc_sum;
			err_ratio_16 = crc_err_sum_16 / crc_sum_16;
			
		}
		else 
		{
			pass_head_flag ++;
		}
	}	
}
extern uint8_t JudgeReveice_DMA_DIR_Flag ;
extern unsigned char JudgeReceiveBuffer_1[JudgeBufBiggestSize];
extern uint16_t Receive_Judge_Date_Len;
void JudgeReceive_task(void)
{
	while(1)
	{
		ulTaskNotifyTake( pdTRUE , portMAX_DELAY ); 	
		JudgeReveice_Flag = 0;
		JudgeDataSolve(JudgeReceiveBuffer_1,Receive_Judge_Date_Len);
	}
 
}
