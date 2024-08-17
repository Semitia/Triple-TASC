/**
 * @file clock.c
 * @author Alexavier
 * @brief used to record the time
 * @version 0.1
 * @date 2024-07-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "clock.h"
#include "n32l40x_tim.h"
#include <stdint.h>

SysTime_t sys_time = {0};
uint16_t last_cnt = 0;
uint16_t getTimeCNT(void)
{
	return TIM_GetCnt(CLK_TIMx);
}

/**
 * @brief update the system time
            适用于定时器不会重装载、中断，CNT会溢出
 */
void sysTimeUpdate(void)
{
    volatile uint16_t cnt_now = getTimeCNT();
    static uint32_t CNT_TEMP1, CNT_TEMP2;
    uint32_t dt = (uint32_t)((cnt_now - last_cnt)*TIME_UNIT);

    CNT_TEMP1 = (dt + sys_time.us) / 1000;
    CNT_TEMP2 = (dt + sys_time.us) - CNT_TEMP1 * 1000;
    sys_time.ms += CNT_TEMP1;
    sys_time.us = CNT_TEMP2;
    if (sys_time.ms >= 1000)
    {
        sys_time.s += 1;
        sys_time.ms -= 1000;
    }
    last_cnt = cnt_now;
}

/**
 * @brief 适用于定时器会触发中断，CNT会重装载，不会溢出
 */
void sysTimeUpdateIRQ(void)
{
    volatile uint16_t cnt = getTimeCNT();
    uint32_t dt = (uint32_t)((cnt - last_cnt + TIMx_ARR) * TIME_UNIT);
    
    sys_time.us += dt;
    if (sys_time.us >= 1000)
    {
        sys_time.ms += sys_time.us / 1000;
        sys_time.us = sys_time.us % 1000;
    }
    if (sys_time.ms >= 1000)
    {
        sys_time.s += sys_time.ms / 1000;
        sys_time.ms = sys_time.ms % 1000;
    }
}

/**
 * @brief 计算两次调用之间的时间差
 * @param[in] cnt_last 上一次计数值
 */
float getDeltaT(uint32_t cnt_last)
{
    volatile uint32_t cnt_now = getTimeCNT();
		// 无符号整数的溢出在C语言中是定义良好的,即使计数器reload也能得到正确结果
    float dt = ((uint16_t)(cnt_now - cnt_last)) / ((float)(SAMP_FREQ));
    return dt;
}

/**
 * @brief 获取系统时间
 * @return SysTime_t 系统时间
 */
SysTime_t getSysTime(void)
{
    sysTimeUpdate();
    return sys_time;
}

float getSysTime_s(void)
{
    sysTimeUpdate();
    return (sys_time.s + sys_time.ms/1000.0f + sys_time.us/1000000.0f);
}

/**
 * @brief 微秒延时函数
 * 
 * @param us 
 */
void delayUs(uint32_t us)
{
    uint16_t cnt_start = getTimeCNT();
    while((getTimeCNT() - cnt_start) < (us / TIME_UNIT))
        ;
}

/**
 * @brief 毫秒延时函数
 * 
 * @param ms 
 */
void delayMs(uint32_t ms)
{
    SysTime_t start_time = getSysTime();
    while (getSysTime().ms - start_time.ms < ms)
        ;
}
