#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define __IO    volatile 
typedef __IO uint16_t vu16;

extern ADC_HandleTypeDef hadc1;
extern u16 sensor_ID;
void Error_Handler(void);
// void sendCanMsg(uint8_t *msg, uint8_t len);
void CAN_SendState(uint8_t *msg, uint8_t len);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
