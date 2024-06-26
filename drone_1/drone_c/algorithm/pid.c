/* include */
#include "stm32f4xx.h"
#include "math_cal.h"
#include "stdint.h"
#include "pid.h"

/*define*/
 

/*  pid init */
//static void pid_param_init(
//	Pid_Typedef*   pid,
//	uint32_t maxout,
//	uint32_t intergral_limit,
//	float    kp,
//	float    ki,
//	float    kd)
//{
//  pid->IMax = intergral_limit;
//  pid->OutMax = maxout;

//  pid->P = kp;
//  pid->I = ki;
//  pid->D = kd;
//}

/*  pid reset */
void pid_reset(Pid_Typedef *pid)
{
  pid->POut = 0;
  pid->IOut = 0;
  pid->DOut = 0;
  pid->Out  = 0;
}

/**********************************************************************************************************
*函 数 名: PID_Calc
*功能说明: PID+各种优化
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
float PID_Calc(Pid_Typedef *P)
{
	P->LastError = P->PreError;//误差
	P->PreError = P->SetPoint - P->ActualValue;//微分
	P->SumError += P->PreError;
	
	//积分
	if(P->SumError > P->IMax)
		P->SumError = P->IMax;
	else if(P->SumError < -P->IMax)
		P->SumError = -P->IMax;

	
	P->POut = P->P * P->PreError;
	P->IOut = P->I * (P->SumError+P->LastError)/2;
	P->DOut = P->D * (P->PreError-P->LastError);
	
	P->Out = P->POut + P->IOut + P->DOut;  
	
	return P->Out;

}

/**********************************************************************************************************
*函 数 名: PID_Calc
*功能说明: PID+各种优化
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
float PID_Calc_test(Pid_Typedef *P)
{
	P->LastError = P->PreError;
	P->PreError = P->SetPoint - P->ActualValue;
	if((ABS(P->PreError)< P->DeadZone ))   //死区控制
	{
		P->PreError = 0.0f;			
	}
		
	//微分先行
	float DM = P->D*(P->Out - P->Out_last);   //微分先行

	//变速积分   (积分分离)
	if(ABS(P->PreError) < P->I_L )			
	{
			 
		P->SumError += (P->PreError+P->LastError)/2;    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
	}
	else if( ABS(P->PreError) < P->I_U )
	{
	  //梯形积分
		P->SumError += (P->PreError+P->LastError)/2*(ABS(P->PreError) - P->I_L)/(P->I_U - P->I_L);    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
	}
	else
	P->SumError = 0;
			
	P->POut = P->P * P->PreError;
	
	P->IOut = P->I * P->SumError;
			
			//不完全微分
	P->DOut_last = P->DOut; 
	P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
	
	P->Out_last  = P->Out;
	P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
	
	return P->Out;

}

/**********************************************************************************************************
*函 数 名: PID_Concatenation_cal
*功能说明: 串联pid
*形    参: Pid_Typedef *Pos,Pid_Typedef *Speed,位置环和速度环双环控制
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
float PID_Concatenation_cal(Pid_Typedef *Pos,Pid_Typedef *speed){
	speed->SetPoint =PID_Calc(Pos);
	return LIMIT_MAX_MIN(PID_Calc(speed),speed->OutMax,-speed->OutMax);
}


/*
模糊pid模块
*/
/****************************************************************模糊pid部分*/
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3
//Kp规则矩阵 
static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PL,	PL,	PM,	PS,	PS,	ZE,	NS,
	PM,	PM,	PM,	PS,	ZE,	NS,	NS,
	PM,	PM,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	PS,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	ZE,	NM,	NM,	NM,	NL,	NL
};
//Ki规则矩阵 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NM,	NS,	ZE,	PS,	PM,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PL,
	ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL
};
//Kd规则矩阵 
static const float fuzzyRuleKd[7][7]={
	PS,	NS,	NL,	NL,	NL,	NM,	PS,
	PS,	NS,	NL,	NM,	NM,	NS,	ZE,
	ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
	ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
	PL,	NS,	PS,	PS,	PS,	PS,	PL,
	PL,	PM,	PM,	PM,	PS,	PS,	PL
};
 

 //关键算法
void fuzzy( FuzzyPID*  fuzzy_PID)//模糊算法
{
     float e = fuzzy_PID ->PreError/ fuzzy_PID->stair;//输入E动态比较
	   float ec = (fuzzy_PID ->Out - fuzzy_PID ->Out_last) / fuzzy_PID->stair;
     short etemp,ectemp;//输入E,EC后的判断值隶属
     float eLefttemp,ecLefttemp;    //隶属度
     float eRighttemp ,ecRighttemp; 
 
     short eLeftIndex,ecLeftIndex;  //标签
     short eRightIndex,ecRightIndex;

	  //模糊化,由大到小
     if(e>=PL)
			 etemp=PL;//超出范围
		 else if(e>=PM)
			 etemp=PM;
		 else if(e>=PS)
			 etemp=PS;
		 else if(e>=ZE)
			 etemp=ZE;
		 else if(e>=NS)
			 etemp=NS;
		 else if(e>=NM)
			 etemp=NM;
		 else if(e>=NL)
			 etemp=NL;
		 else 
			 etemp=2*NL;
 
		 if( etemp == PL)
		{
		 //计算E隶属度
				eRighttemp= 0 ;    //右溢出
				eLefttemp= 1 ;
			
     //计算标签
	   eLeftIndex = 6 ;      
	   eRightIndex= 6 ;
			
		}else if( etemp == 2*NL )
    {

			//计算E隶属度
				eRighttemp = 1;    //左溢出
				eLefttemp = 0;
	
     //计算标签
	   eLeftIndex = 0 ;       
	   eRightIndex = 0 ;
			
		}	else 
    {

			//计算E隶属度
				eLefttemp=(e-etemp);  //线性函数作为隶属函数
				eRighttemp=(1- eLefttemp);
			
     //计算标签
	   eLeftIndex =(short) (etemp-NL);       //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
	   eRightIndex=(short) (eLeftIndex+1);
			
		}		
	   
		
		 if(ec>=PL)
			 ectemp=PL;
		 else if(ec>=PM)
			 ectemp=PM;
		 else if(ec>=PS)
			 ectemp=PS;
		 else if(ec>=ZE)
			 ectemp=ZE;
		 else if(ec>=NS)
			 ectemp=NS;
		 else if(ec>=NM)
			 ectemp=NM;
		 else if(ec>=NL)
			 ectemp=NL;
		 else 
			 ectemp=2*NL;
		 
	  
   if( ectemp == PL )
	 {
    //计算EC隶属度		 
		 ecRighttemp= 0 ;      //右溢出
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //计算EC隶属度		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //计算EC隶属度		 
		 ecLefttemp=(ec-ectemp);
		 ecRighttemp=(1- ecLefttemp);
			
		 ecLeftIndex =(short) (ectemp-NL);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }	

 
/*************************************反模糊*************************************/
	fuzzy_PID->dKp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->dKi = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	fuzzy_PID->dKd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
 
}


float FuzzyPID_Calc(FuzzyPID *P,float SetPoint,float ActualValue)//模糊计算
{
	
	  P->LastError = P->PreError;
	  
	  if((ABS(P->PreError)< P->DeadZone ))   //死区控制
		{
			P->PreError = 0.0f;			
		}
		else
		{
			P->PreError = SetPoint - ActualValue;
		}
		
		fuzzy(P);      //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值
	
    float Kp = P->Kp0 + P->dKp , Ki = P->Ki0 + P->dKi , Kd = P->Kd0 + P->dKd ;   //PID均模糊
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 + P->dKd ;           //仅PD均模糊
//	float Kp = P->Kp0 + P->dKp , Ki = P->Ki0  , Kd = P->Kd0 ;                    //仅P均模糊

		
		
		      //微分先行
		float DM = Kd*(P->Out - P->Out_last);   //微分先行	
         //变速积分
    if(ABS(P->PreError) < P->I_L )			
		{
	       //梯形积分
		P->SumError += (P->PreError+P->LastError)/2;    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);
		}
		 else if( ABS(P->PreError) < P->I_U )
		{
	       //梯形积分
		P->SumError += (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);    
		P->SumError = LIMIT_MAX_MIN(P->SumError,P->IMax,- P->IMax);		
		}
			
		P->POut = Kp * P->PreError;
		
		P->IOut = Ki * P->SumError;
		    
		    //不完全微分
		P->DOut_last = P->DOut; 
		P->DOut = DM * P->RC_DF + P->DOut_last * ( 1 - P->RC_DF );    
		
		P->Out_last  = P->Out;
		
		P->Out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
		
    return P->Out;                             

}


//模糊PID初始化函数
static void Fuzzy_pid_param_init(
	FuzzyPID *   pid,
	uint32_t OutMax,
	uint32_t IMax,
	float    Kp0,
	float    Ki0,
	float    Kd0,
	float		 dKp,
	float		 dKi,
	float		 dKd,
	float 	 I_U,
	float    I_L,
	float    RC_DM,
	float 	 RC_DF,	
	float 	 stair,
	float 	 Kp_stair,
	float 	 Ki_stair,
	float 	 Kd_stair)
{
  pid->IMax = IMax;
  pid->OutMax = OutMax;

  pid->Kp0 = Kp0;
  pid->Ki0 = Ki0;
  pid->Kd0 = Kd0;
	
	pid->dKp = dKp;
	pid->dKi = dKi;
	pid->dKd = dKd;
	
	pid->I_U = I_U;
	pid->I_L = I_L;
	
	pid->stair = stair;
	pid->Kp_stair = Kp_stair;
	pid->Ki_stair = Ki_stair;
	pid->Kd_stair = Kd_stair;
	
	pid->RC_DF = RC_DF;
	pid->RC_DM = RC_DM;
}
//模糊PID重置函数
static void Fuzzy_pid_reset(FuzzyPID *pid)
{
  pid->dKp = 0;
  pid->dKi = 0;
  pid->dKd = 0;
  
  pid->POut = 0;
  pid->IOut = 0;
  pid->DOut = 0;
  pid->Out  = 0;
	pid->Out_last = 0;
}
//模糊PID结构体初始化从
void Fuzzy_PID_struct_init(
	FuzzyPID *   pid,
	uint32_t OutMax,
	uint32_t IMax,
	float    Kp0,
	float    Ki0,
	float    Kd0,
	float		 dKp,
	float		 dKi,
	float		 dKd,
	float 	 I_U,
	float    I_L,
	float    RC_DM,
	float 	 RC_DF,	
	float 	 stair,
	float 	 Kp_stair,
	float 	 Ki_stair,
	float 	 Kd_stair)
{
  pid->fuzzy_param_init = Fuzzy_pid_param_init;
  pid->fuzzy_pid_reset  = Fuzzy_pid_reset;

  pid->fuzzy_param_init(pid ,OutMax ,IMax ,Kp0 ,Ki0 ,Kd0 ,dKp ,dKi ,dKd ,I_U ,I_L ,RC_DM ,RC_DF ,stair ,Kp_stair ,Ki_stair ,Kd_stair);
  pid->fuzzy_pid_reset(pid);
}
/* end */















