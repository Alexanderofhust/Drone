/**
 ******************************************************************************
 * @file	 SignalGenerator.c
 * @brief    信号发生器定义
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "SignalGenerator.h"
#include "main.h"
void StepInit(StepFunction *step, float initial_value, float step_amplitude, float start_time)
{
    step->start_time = start_time;
    step->time = 0;
    step->initial_value = initial_value;
    step->step_amplitude = step_amplitude;
}
float StepRun(StepFunction *step, float delta_t)
{
    step->time += delta_t;
    if (step->time > step->start_time)
    {
        return step->initial_value + step->step_amplitude;
    }
    else
    {
        return step->initial_value;
    }
}

void SinInit(SinFunction *sin_function, float amplitude, float start_time, uint16_t T)
{
    sin_function->amplitude = amplitude;
    sin_function->start_time = start_time;
    sin_function->cycle_count = 0;
    sin_function->T = T;
    sin_function->w = 2 * PI * 1000.0f / (float)T;
    sin_function->time = 0;
}

float SinRun(SinFunction *sin_function, float delta_t)
{
    sin_function->time += delta_t;
    if (sin_function->time < sin_function->start_time)
    {
        return 0.0f;
    }
    float angle = sin_function->w * (sin_function->time - sin_function->start_time);

    if (angle > 2 * PI * (sin_function->cycle_count + 1))
    {
        sin_function->cycle_count++;
    }

    return sin_function->amplitude * arm_sin_f32(angle);
}

//void SawToothInit(SawToothWave *saw_tooth_wave, float amplitude, float start_time, uint16_t T, float initial_value)
//{
//    saw_tooth_wave->T = T;
//    saw_tooth_wave->amplitude = amplitude;
//    saw_tooth_wave->start_time = start_time;
//    saw_tooth_wave->time = 0;
//    saw_tooth_wave->cycle_count = 0;
//    saw_tooth_wave->initial_value = initial_value;
//    saw_tooth_wave->out = initial_value;
//    saw_tooth_wave->slope = amplitude * 1000.0f / (float)T;
//}

//float SawWaveRun(SawToothWave *saw_tooth_wave, float delta_t)
//{
//    saw_tooth_wave->time += delta_t;
//    if (saw_tooth_wave->time < saw_tooth_wave->start_time)
//    {
//        saw_tooth_wave->out = saw_tooth_wave->initial_value;
//        return saw_tooth_wave->initial_value;
//    }

//    if ((saw_tooth_wave->time - saw_tooth_wave->start_time) > (saw_tooth_wave->cycle_count + 1) * (float)saw_tooth_wave->T / 1000.0f)
//    {
//        saw_tooth_wave->cycle_count++;
//        saw_tooth_wave->out = saw_tooth_wave->initial_value;
//    }

//    saw_tooth_wave->out += saw_tooth_wave->slope * delta_t;
//    return saw_tooth_wave->out;
//}
extern TIM_HandleTypeDef htim3;

//本程序适合在1000hz中运行
/**********************************************************************************************************
*函 数 名: SawToothInit
*功能说明: 初始化saw结构体
*形    参: SawToothWave *saw_tooth_wave  锯齿波结构体，
					 startup_freq  初始频率
  *        amplitude     电压幅度
*返 回 值: 无
**********************************************************************************************************/
void SawToothInit(SawToothWave *saw_tooth_wave,uint16_t startup_freq,float amplitude)
{
		saw_tooth_wave->num = 0;//计数器
		saw_tooth_wave->ARR = 1000/startup_freq;//初始频率为1hz,startup_freq默认为1
		saw_tooth_wave->freq = 1;//频率
		saw_tooth_wave->circle = 0;//特定频率下循环次数设定
		saw_tooth_wave->amplitude = amplitude;
	
}


//请放到对应计时器中断中使用
//microstep是每次循环之后增加的频率
/**********************************************************************************************************
*函 数 名: SawWave
*功能说明: 请放到对应计时器中断中使用，生成可变频率的输出值
*形    参: SawToothWave *saw_tooth_wave
*        microstep    该输出后下一次频率增加值，为0保持恒定频率
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/

void SawWave(SawToothWave *saw_tooth_wave,float microstep)
{
   	saw_tooth_wave->num = saw_tooth_wave->num + 1;
		saw_tooth_wave->out = saw_tooth_wave->amplitude * saw_tooth_wave->freq * saw_tooth_wave->num/1000;
		//超过1s之后，循环++，循环5次后，频率++
		if(saw_tooth_wave->num >= saw_tooth_wave->ARR)
		{
				saw_tooth_wave->circle++;
				saw_tooth_wave->out = 0;
				if(saw_tooth_wave->circle>=1)
				{
						saw_tooth_wave->freq+=microstep;		
						saw_tooth_wave->ARR = (int)(1000/saw_tooth_wave->freq);
//						if(saw_tooth_wave->freq >= saw_tooth_wave->maxfreq)
//							saw_tooth_wave->freq = saw_tooth_wave->maxfreq;
						saw_tooth_wave->circle = 0;
				}
				
				saw_tooth_wave->num = 0;
		}

}
/**********************************************************************************************************
*函 数 名: SawWaveRun
*功能说明: 启动生成锯齿波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void SawWaveRun(void)
{
		HAL_TIM_Base_Start_IT(&htim3);
}
/**********************************************************************************************************
*函 数 名: SawWaveStop
*功能说明: 停止生成锯齿波并且恢复初值
*形    参: SawToothWave *saw_tooth_wave 
  *        startup_freq    初始频率
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
void SawWaveStop(SawToothWave *saw_tooth_wave,uint16_t startup_freq)
{
		HAL_TIM_Base_Stop_IT(&htim3);
		saw_tooth_wave->num = 0;//计数器
		saw_tooth_wave->ARR = 1000/startup_freq;//初始频率为1hz
		saw_tooth_wave->freq = 1;//频率
		saw_tooth_wave->circle = 0;//特定频率下循环次数设定
}