#ifndef __OPAMP_H__
#define __OPAMP_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "n32l40x_adc.h"
#include "n32l40x_gpio.h"
#include "n32l40x_opamp.h"
#include "n32l40x_rcc.h"

#define OPAMP_NORMAL

extern uint16_t adc_value[2];
	
void ns_adc_init(void);
uint16_t ns_get_adcvalue(uint8_t ADC_Channel);

void ns_port_init(void);
void ns_rcc_init(void);
		
void ns_opamp_init(void);
void ns_opamp_normal(void);
void ns_opamp_pga(void);
void ns_opamp_follow(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __OPAMP_H__ */
