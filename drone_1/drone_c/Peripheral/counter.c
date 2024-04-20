/**
 ******************************************************************************
 * @file    counter.c
 * @brief   ��ʱ�������ڼ��������ʱ������ʱ
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "main.h"
#include "counter.h"
/*  ��ʱ��������  */
#include "main.h"
uint32_t LastCNT32; // ��һ��64λֵ
//SysTime_t SysTime;  //ϵͳʱ�䣬�룬���룬΢��

extern TIM_HandleTypeDef htim2;

/**
 * @brief �������ε���֮���ʱ���
 * @param[in] cnt_last ��һ�μ���ֵ
 */
float GetDeltaT(uint16_t *cnt_last)
{
    volatile uint16_t cnt_now=	__HAL_TIM_GET_COUNTER(&htim2);
    float dt = ((uint16_t)(cnt_now - *cnt_last)) / 1000000;//dt��λ��us��������cnt����ǧ��������
    *cnt_last = cnt_now;

    return dt;
}

///**
// * @brief �������ε���֮���ʱ���(˫����)
// * @param[in] cnt_last ��һ�μ���ֵ
// */
//double GetDeltaT64(uint32_t *cnt_last)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(COUNTER_SAMPLING));
//    *cnt_last = cnt_now;

//    return dt;
//}

///**
// * @brief ������ʱ�� s/ms/us
// * @param[in] void
// */
//void SysTimeUpdate(void)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    static uint64_t CNT_TEMP1, CNT_TEMP2;
//    uint32_t diff_cnt = cnt_now - LastCNT32;

//    CNT_TEMP1 = (diff_cnt + SysTime.us) / 1000;             // ms
//    CNT_TEMP2 = (diff_cnt + SysTime.us) - CNT_TEMP1 * 1000; // us
//    SysTime.ms += CNT_TEMP1;
//    SysTime.us = CNT_TEMP2;
//    if (SysTime.ms >= 1000)
//    {
//        SysTime.s += 1;
//        SysTime.ms -= 1000;
//    }

//    LastCNT32 = cnt_now;
//}

/**
 * @brief ��ȡ�ӳ���ʼ�����ڵ�ʱ�� /s
 * @param[in] void
 */
//float GetTime_s(void)
//{
//    SysTimeUpdate();

//    float Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

//    return Timelinef32;
//}

///**
// * @brief ��ȡ�ӳ���ʼ�����ڵ�ʱ�� /s
// * @param[in] void
// */
//float GetTime_ms(void)
//{
//    SysTimeUpdate();

//    float Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

//    return Timelinef32;
//}

///**
// * @brief ��ȡ�ӳ���ʼ�����ڵ�ʱ�� /us
// * @param[in] void
// */
//uint64_t GetTime_us(void)
//{
//    SysTimeUpdate();

//    uint64_t Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

//    return Timelinef32;
//}

///**
// * @brief ��ʱ /us
// * @param[in] void
// */
//void Delay(uint32_t Delay)
//{
//    volatile uint32_t tickstart = COUNTER_TIMx->CNT;
//    uint32_t wait = Delay;

//    while ((COUNTER_TIMx->CNT - tickstart) < wait)
//    {
//    }
//}

///**
// * @brief ���㵽����ʱ��ֵ�����ڵ�ʱ��
// * @param[in] cnt_last ��һ�μ���ֵ
// */
//float GetDeltaTtoNow(uint32_t *cnt_last)
//{
//    volatile uint32_t cnt_now = COUNTER_TIMx->CNT;
//    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(COUNTER_SAMPLING));

//    return dt;
//}
