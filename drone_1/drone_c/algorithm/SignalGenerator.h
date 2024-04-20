#ifndef _SIGNAL_GENERATOR_H
#define _SIGNAL_GENERATOR_H

#include "stdint.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "main.h"

/* 阶跃函数 */
typedef struct StepFunction
{
    float initial_value;
    float step_amplitude; //阶跃幅值
    float start_time;
    float time;
} StepFunction;

void StepInit(StepFunction *step, float initial_value, float step_amplitude, float start_time);
float StepRun(StepFunction *step, float delta_t);

/* 正弦波函数 */
typedef struct SinFunction
{
    float start_time; //开始时间
    float time;
    float amplitude;      //幅值
    float w;              //角速度
    uint16_t T;           //周期 * 0.001s
    uint16_t cycle_count; //经过了几个周期
} SinFunction;

void SinInit(SinFunction *sin_function, float amplitude, float start_time, uint16_t T);
float SinRun(SinFunction *sin_function, float delta_t);

/* 锯齿波发生函数 */
//typedef struct SawToothWave
//{
//    uint16_t T;          //单个周期长度
//    float initial_value; //初值
//    float amplitude;     //幅值
//    float start_time;    //开始时间
//    float time;
//    float slope;          //斜率
//    float out;            //输出
//    uint16_t cycle_count; //经过了几个周期
//} SawToothWave;

//void SawToothInit(SawToothWave *saw_tooth_wave, float amplitude, float start_time, uint16_t T, float initial_value);
//float SawWaveRun(SawToothWave *saw_tooth_wave, float delta_t);


typedef struct SawToothWave
{
		uint32_t num;//计数器
		uint16_t ARR;//计数毫秒数后刷新，默认为1000，方便直接修改频率
		float freq;//频率
		short freqmax;//最大频率
		short circle;//特定频率下循环次数设定
		float amplitude;     //幅值
		float out;//输出值
		TIM_HandleTypeDef htim;//默认使用定时器3
} SawToothWave;

void SawToothInit(SawToothWave *saw_tooth_wave,uint16_t startup_freq,float amplitude);
void SawWave(SawToothWave *saw_tooth_wave,float microstep);
void SawWaveStop(SawToothWave *saw_tooth_wave,uint16_t startup_freq);
void SawWaveRun(void);


#ifndef TRUE
#define TRUE 1
#endif // !
#ifndef FALSE
#define FALSE 0
#endif // !
//#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef PI_2
#define PI_2 6.2831853072f
#endif

/* 普通延时 */
//void delay_ms(unsigned long t);
//void delay_us(unsigned long t);


#endif // !_SIGNAL_H
