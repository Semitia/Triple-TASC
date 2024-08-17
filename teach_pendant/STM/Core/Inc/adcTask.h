#ifndef __ADCTASK_H
#define __ADCTASK_H
#include "main.h"

#define ADC1_CHANNEL_CNT 2
#define ADC1_CHANNEL_FRE 3	

extern uint16_t value[ADC1_CHANNEL_CNT];
extern uint16_t adc1_val_buf[ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE]; 
extern uint32_t adc1_sum_val[ADC1_CHANNEL_CNT]; 
extern uint16_t value[ADC1_CHANNEL_CNT];
void adcTask(void *argument);

#endif // __ADCTASK_H

