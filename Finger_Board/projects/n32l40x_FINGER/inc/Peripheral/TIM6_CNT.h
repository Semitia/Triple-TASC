/**
 * @file TIM6_CNT.h
 * @author Alexavier
 * @brief use TIM6 as the clock, while generating interrupts every 1ms
 * @version 0.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __TIM6_CNT_H
#define __TIM6_CNT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"

typedef uint16_t SysCNT_t;

void tim6Init(void);

#ifdef __cplusplus
}
#endif
#endif  // __TIM6_CNT_H
