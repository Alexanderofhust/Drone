/* include */
#include "stm32f4xx.h"
#include "zerocheck.h"

/* get zerocheck data */
/* λ��ʽ���ٶ�ʽ������ */
/* ʹ�ô˺���ǰҪ������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue */
/* paramter:	
		ZeroCheck_Typedef *Zero 	������ṹ��
			Zero->ActualValue 	��ʾ�������ǰֵ
			Zero->LastValue 		��ʾ�������һ��ֵ
			Zero->CountCycle 		��ʾ���������ʱԽ��ֵ(����������)
			Zero->PreError 			��ʾ�������ֵ
		float value  							������� */
float Get_ZeroCheck_Value_Interface(ZeroCheck_Typedef *zero,float value)
{
	zero->ActualValue=value;
	
	zero->PreError=zero->ActualValue-zero->LastValue; //�������ֵ
	
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
	//�����һ����8000��һ����001������μ�ȥ�ϴ�С��-0.7*8192����Ȧ��ȴ�����ӵ�
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

/* ��������ֵ���� */
float ResetZeroCheck(ZeroCheck_Typedef *zeroCheck)
{
	float temp = zeroCheck->CountCycle;
	ZeroCheck_Init_Interface(zeroCheck,temp,0);
	return 1;
}


/* end */

