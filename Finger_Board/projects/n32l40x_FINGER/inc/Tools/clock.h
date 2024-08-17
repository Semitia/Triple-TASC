/**
 * @file clock.h
 * @author Alexavier
 * @brief used to record the time
 * @version 0.1
 * @date 2024-07-17
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __CLOCK_H
#define __CLOCK_H
#ifdef __cplusplus
extern "C" {
#endif

#include "TIM6_CNT.h"

#define CLK_TIMx TIM6
#define TIME_UNIT 1.0f //us
#define SAMP_FREQ (1000000.0f/TIME_UNIT)
#define TIMx_ARR 1000

//系统时间结构体，用来记录系统运行时间
typedef struct SysTime_t
{
    uint32_t s;
    uint32_t ms;
    uint32_t us;
} SysTime_t;

extern SysTime_t sys_time;
void sysTimeUpdate(void);
float getSysTime_s(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);
SysTime_t getSysTime(void);
void sysTimeUpdateIRQ(void);
float getDeltaT(uint32_t cnt_last);

#ifdef __cplusplus
}
#endif
#endif // __CLOCK_H
