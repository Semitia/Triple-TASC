#include "adcTask.h"

uint16_t adc1_val_buf[ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE]; 
uint32_t adc1_sum_val[ADC1_CHANNEL_CNT] = {0}; 
uint16_t value[ADC1_CHANNEL_CNT] = {0};

void adcTask(void *argument) {
		uint16_t i,j;
		if(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK){
		 Error_Handler(); 
		}
    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1_val_buf, (ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE)) != HAL_OK) {
        /* Start Conversation Error */
        Error_Handler(); 
    }
    while(1) {
//				// __HAL_DMA_ENABLE(&hdma_adc1);
//        for(i=0;i<ADC1_CHANNEL_CNT;i++){
//            adc1_sum_val[i] = 0;
//        }
//        for(i=0;i<ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE;i++){
//            j = i%ADC1_CHANNEL_CNT;
//            adc1_sum_val[j] += adc1_val_buf[i];
//        }
//        for(i=0;i<ADC1_CHANNEL_CNT;i++){
//            value[i] = adc1_sum_val[i]/ADC1_CHANNEL_FRE;
//        }
				//			HAL_ADC_Stop_DMA(&hadc1);
        osDelay(20);
    }
}

