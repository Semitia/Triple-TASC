/**
 * @file TIM1_PWM.h
 * @author Alexavier
 * @brief use TIM1 to generate PWM signal
 * @version 0.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __TIM1_PWM_H
#define __TIM1_PWM_H
#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"
#include "user_lib.h"

void timInit(TIM_Module* TIMx);

#ifdef __cplusplus
}
#endif
#endif  // __TIM1_PWM_H
