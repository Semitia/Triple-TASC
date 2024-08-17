/**
 * @file n_cg_opamp.c
 * @author Nations fae Team
 * @version v0.0.1
 *
 * @copyright Copyright (c) 2022, Nations Technologies Inc. All rights reserved.
 */
#include "OPAMP.h"

/** @addtogroup OPA_PGA
 * @{
 */

ADC_InitType ADC_InitStructure;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void OPA_Configuration(void);
void COMP_Configuratoin(void);
void ADC_SampleConfig(void);
void TIM_PwmConfig(TIM_Module* TIMx);
void TIM_AllPwmOpen(TIM_Module* TIMx);
void TIM_AllPwmShut(TIM_Module* TIMx);
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3);

/**
 * @brief   Main program,Test PGA is work ok? Opa out Pin can view by scope
 */
uint8_t Gain = 2, BakGain = 0;
void ns_opamp_init(void) {
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* OPA configuration ------------------------------------------------------*/
    OPA_Configuration();

    /* ADC configuration ------------------------------------------------------*/
    ADC_SampleConfig();

    /* TIMx configuration ------------------------------------------------------*/
    TIM_PwmConfig(TIM1);
		TIM_PwmConfig(TIM8);

    /*test*/
    //    TIM_AllPwmOpen(TIM1);

    //    OPAMP_SetPgaGain(OPAMP1, OPAMP_CS_PGA_GAIN_4);
    //    OPAMP_SetPgaGain(OPAMP2, OPAMP_CS_PGA_GAIN_4);
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void) {
    /* Enable GPIOA clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);

    /* Enable COMP OPA clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_OPAMP, ENABLE);

    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);

    /* selsect HSE as RCC ADC1M CLK Source */
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);

    /* RCC_ADCHCLK_DIV16*/
    RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV10);

    /* Enable TIMx clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void) {
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Analog;

    /* OPA VM   OPA1_VM, OPA2_VM,
                PA3      PA5               as analog inputs */
    OPAMP_SetVmSel(OPAMP1, OPAMP1_CS_VMSEL_PA3);
    OPAMP_SetVmSel(OPAMP2, OPAMP2_CS_VMSEL_PA5);
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /* OPA VP   OPA1_VP, OPA2_Vp,
                PA4      PA7               as analog inputs */
    OPAMP_SetVpSel(OPAMP1, OPAMP1_CS_VPSEL_PA7);
    OPAMP_SetVpSel(OPAMP2, OPAMP2_CS_VPSEL_PA4);
    GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_7;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /*OPA OUT   OP1_out,  OP2_out,
                PA2      PA6               as analog output */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Analog;
    GPIO_InitStructure.Pin       = GPIO_PIN_2 | GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /* TIM1 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM1;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin            = GPIO_PIN_1;
		GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_TIM1;
		GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
		
}

/**
 * @brief  Configures the Opa.
 */
void OPA_Configuration(void) {
    OPAMP_InitType OPAMP_Initial;
    OPAMP_StructInit(&OPAMP_Initial);
    OPAMP_Initial.Mod = OPAMP_CS_EXT_OPAMP;
    // OPAMP_Initial.Gain           = OPAMP_CS_PGA_GAIN_2;
    // OPAMP_Initial.HighVolRangeEn = ENABLE;
    OPAMP_Initial.TimeAutoMuxEn = DISABLE;
    /*configure opa1*/
    OPAMP_Init(OPAMP1, &OPAMP_Initial);
    OPAMP_Enable(OPAMP1, ENABLE);
    /*configure opa2*/
    OPAMP_Init(OPAMP2, &OPAMP_Initial);
    OPAMP_Enable(OPAMP2, ENABLE);
}

/**
 * @brief  Configures the Adcx.
 */
void ADC_SampleConfig(void) {
    ADC_InitType ADC_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* ADC registers reenable */
    ADC_DeInit(ADC);

    /*ADC configuration*/
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_INJ_CONV_T1_CC4;
    // ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIG_INJ_CONV_EXT_INT15_TIM8_CC4;
		ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 2;
    ADC_Init(ADC, &ADC_InitStructure);

    /*ADCx Injected conversions configuration,Config Sampling Time*/
    ADC_ConfigInjectedSequencerLength(ADC, 2);
    ADC_ConfigInjectedChannel(ADC, ADC_CH_3_PA2, 1, ADC_SAMP_TIME_1CYCLES5);  // OPA1
    ADC_ConfigInjectedChannel(ADC, ADC_CH_7_PA6, 2, ADC_SAMP_TIME_1CYCLES5);  // OPA2

    /*ADC TrigInJectConv Enable*/
    ADC_EnableExternalTrigInjectedConv(ADC, ENABLE);

    /*NVIC Initial*/
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /*Enable the ADC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);
    /* Check ADC Ready */
    while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY) == RESET);

    /*Start Calibration*/
    ADC_StartCalibration(ADC);
    /* Wait for the end of ADCs calibration */
    while (ADC_GetCalibrationStatus(ADC)) {
    }

    /*Enable Temp and Vrefint*/
    ADC_EnableTempSensorVrefint(ENABLE);

    /*ADC1 Injected group of conversions end and Analog Watchdog interruptsenabling */
    ADC_ConfigInt(ADC, ADC_INT_JENDC | ADC_INT_AWD, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8.
 */
void TIM_PwmConfig(TIM_Module* TIMx) {
    TIM_TimeBaseInitType TIMx_TimeBaseStructure;
    OCInitType TIMx_OCInitStructure;
    uint16_t TimerPeriod = 0;

    TimerPeriod = (SystemCoreClock / 20000) - 1;

    /*Time Base configuration*/
    TIM_DeInit(TIMx);
    TIM_InitTimBaseStruct(&TIMx_TimeBaseStructure);
    TIMx_TimeBaseStructure.Prescaler = 0;
    TIMx_TimeBaseStructure.CntMode   = TIM_CNT_MODE_CENTER_ALIGN2;  // 01:/\,irq flag only counter down
    TIMx_TimeBaseStructure.Period    = TimerPeriod;                 // PWM_PERIOD;
    TIMx_TimeBaseStructure.ClkDiv    = 0;                           // TIM_CLK_DIV2;
    TIMx_TimeBaseStructure.RepetCnt =
        0;  // REP_RATE;// Initial condition is REP=0 to set the UPDATE only on the underflow
    TIM_InitTimeBase(TIMx, &TIMx_TimeBaseStructure);

    /*Channel 1, 2,3 in PWM mode */
    TIM_InitOcStruct(&TIMx_OCInitStructure);
    TIMx_OCInitStructure.OcMode       = TIM_OCMODE_PWM1;  // when '<' is active,when '>' is inactive
    TIMx_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
    TIMx_OCInitStructure.Pulse        = (TimerPeriod >> 1);  // dummy value
    TIMx_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_HIGH;
    TIMx_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_HIGH;
    TIMx_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_RESET;
    TIMx_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;
    // TIM_InitOc1(TIMx, &TIMx_OCInitStructure);
    // TIM_InitOc2(TIMx, &TIMx_OCInitStructure);
    // TIM_InitOc3(TIMx, &TIMx_OCInitStructure);
    /*Channel 4 Configuration in OC */
    TIMx_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIMx_OCInitStructure.Pulse       = TimerPeriod - 200;
    TIM_InitOc4(TIMx, &TIMx_OCInitStructure);

    /*Enables the TIM1 Preload on CC1,CC2,CC3,CC4 Register */
    TIM_ConfigOc1Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc2Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc3Preload(TIMx, TIM_OC_PRE_LOAD_ENABLE);

    /*Sel Output Trigger*/
    TIM_SelectOutputTrig(TIMx, TIM_TRGO_SRC_OC4REF);  // Master mode select,010:The Update event as trigger output(TRGO)

    /*TIMx counter enable*/
    TIM_Enable(TIMx, ENABLE);
    TIM_EnableCtrlPwmOutputs(TIMx, ENABLE);
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output enable.
 */
void TIM_AllPwmOpen(TIM_Module* TIMx) {
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3EN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc output shut.
 */
void TIM_AllPwmShut(TIM_Module* TIMx) {
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 cc brake
 */
void TIM_Brake(TIM_Module* TIMx) {
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC1EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC2EN));
    TIMx->CCEN &= (uint16_t)(~((uint16_t)TIM_CCEN_CC3EN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC1NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC2NEN));
    TIMx->CCEN |= (uint16_t)(((uint16_t)TIM_CCEN_CC3NEN));
}

/**
 * @brief  Configures the Tim1 or Tim8 set cc
 */
void TIM_DutySet(TIM_Module* TIMx, int16_t duty1, int16_t duty2, int16_t duty3) {
    TIMx->CCDAT1 = duty1;
    TIMx->CCDAT2 = duty2;
    TIMx->CCDAT3 = duty3;
}

/*Irq Adcx Samp value*/
uint16_t adc_value[2] = {0};
static  uint16_t new_adc_value = 0;
#define MASK_AD_BITS 0x0FFF  // min 7bits
#define ALPHA 0.01
/**
 * @brief  Irq with adc1 and adc2
 */
void ADC_IRQHandler(void) {
    if (ADC_GetIntStatus(ADC, ADC_INT_JENDC) == SET) {
			ADC_ClearFlag(ADC, ADC_FLAG_JENDC);
			new_adc_value = ADC_GetInjectedConversionDat(ADC, ADC_INJ_CH_1) & MASK_AD_BITS;
			adc_value[0] = ALPHA * new_adc_value + (1-ALPHA)*adc_value[0];
			new_adc_value  = ADC_GetInjectedConversionDat(ADC, ADC_INJ_CH_2) & MASK_AD_BITS;
			adc_value[1] = ALPHA * new_adc_value + (1-ALPHA)*adc_value[1];
    } else {
        if (ADC_GetIntStatus(ADC, ADC_INT_AWD) == SET)
            ADC_ClearFlag(ADC, ADC_FLAG_AWDG);
    }
}
