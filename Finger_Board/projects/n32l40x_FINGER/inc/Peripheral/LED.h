/**
 * @file LED.h
 * @author Alexavier
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __LED_H__
#define __LED_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"

void LedInit(GPIO_Module* GPIOx, uint16_t Pin);
void LedOn(GPIO_Module* GPIOx, uint16_t Pin);
void LedOff(GPIO_Module* GPIOx, uint16_t Pin);
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);

#ifdef __cplusplus
}
#endif
#endif /* __LED_H__ */
