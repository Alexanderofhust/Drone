#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

typedef struct FeedForward
{
	float param[3];//前馈系数
	float val;
	float val_dot;//一阶差分
	float val_ddot;//二阶差分
	
	float output;
	float Max_Out;
	
	
} FeedForward_Typedef;

float FeedForward_Cal(FeedForward_Typedef *ff,float now_val);
void FeedForwardInit(void);

#endif
