/* include */
#include "stm32f4xx.h"
#include "zerocheck.h"

/* get zerocheck data */
/* 位置式和速度式过零检测 */
/* 使用此函数前要申明对应检测量结构体的 Zero->CountCycle与Zero->LastValue */
/* paramter:	
		ZeroCheck_Typedef *Zero 	过零检测结构体
			Zero->ActualValue 	表示检测量当前值
			Zero->LastValue 		表示检测量上一次值
			Zero->CountCycle 		表示检测量过零时越变值(即计数周期)
			Zero->PreError 			表示检测量差值
		float value  							待检测量 */
float Get_ZeroCheck_Value_Interface(ZeroCheck_Typedef *zero,float value)
{
	zero->ActualValue=value;
	
	zero->PreError=zero->ActualValue-zero->LastValue; //检测量差值
	
	zero->LastValue=zero->ActualValue;
	
	if(zero->PreError>0.7f*zero->CountCycle)
	{
		zero->PreError=zero->PreError-zero->CountCycle;
		zero->Circle--;
	}
	
	if(zero->PreError<-0.7f*zero->CountCycle)
	{
		zero->PreError=zero->PreError+zero->CountCycle;
		zero->Circle++;
	}
	//如果上一次是8000下一次是001，则这次减去上次小于-0.7*8192但是圈数却是增加的
	return zero->ActualValue + zero->Circle * zero->CountCycle;
}

/* zerocheck init */
void ZeroCheck_Init_Interface(ZeroCheck_Typedef *zeroCheck,float CountCycle,float NowValue)
{
	zeroCheck->ActualValue = 0;
	zeroCheck->CountCycle  = CountCycle;
	zeroCheck->LastValue   = NowValue;
	zeroCheck->PreError    = 0;
	zeroCheck->Circle = 0;
}

/* 将过零检测值置零 */
float ResetZeroCheck(ZeroCheck_Typedef *zeroCheck)
{
	float temp = zeroCheck->CountCycle;
	ZeroCheck_Init_Interface(zeroCheck,temp,0);
	return 1;
}


/* end */

