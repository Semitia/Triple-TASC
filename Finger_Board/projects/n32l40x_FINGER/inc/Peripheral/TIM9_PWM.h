/**
 * @file TIM9_PWM.h
 * @author Alexavier
 * @brief use TIM9 to generate PWM signal
 * @version 0.1
 * @date 2024-07-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __TIM9_PWM_H
#define __TIM9_PWM_H
#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"
#include "user_lib.h"

void tim9Init(void);

#ifdef __cplusplus
}
#endif
#endif  // __TIM9_PWM_H
