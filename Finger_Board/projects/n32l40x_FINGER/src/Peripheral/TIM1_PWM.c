/**
 * @file TIM1_PWM.c
 * @author Alexavier
 * @brief use TIM1 to generate PWM signal
 * @version 0.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "TIM1_PWM.h"

void TIM_RCC_Configuration(void);
void TIM_GPIO_Configuration(void);

/**
 * @brief   intial timx program
 */
void timInit(TIM_Module* TIMx) {
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    TIM_RCC_Configuration();
    TIM_GPIO_Configuration();

    /* Time Base configuration */
    TIM_TimeBaseStructure.Prescaler = 7;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.Period    = 999;
    TIM_TimeBaseStructure.ClkDiv    = 0;

    TIM_InitTimeBase(TIMx, &TIM_TimeBaseStructure);

    /* Channel 1, 2 Configuration in PWM mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 500;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_OCInitStructure.OcIdleState = TIM_OC_IDLE_STATE_SET;  // important
    TIM_InitOc1(TIMx, &TIM_OCInitStructure);
    TIM_OCInitStructure.Pulse = 500;
    TIM_InitOc4(TIMx, &TIM_OCInitStructure);

    /* TIM1 counter enable */
    TIM_Enable(TIMx, ENABLE);

    /* TIM1 Main Output Enable */
    TIM_EnableCtrlPwmOutputs(TIMx, ENABLE);
}

/**
 * @brief  Configures the different system clocks.
 */
void TIM_RCC_Configuration(void) {
    /* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_TIM1 | RCC_APB2_PERIPH_AFIO,
                            ENABLE);
    // When debug ,TIM1 and TIM8 stop
    // DBG_ConfigPeriph(DBG_TIM1_STOP | DBG_TIM8_STOP, ENABLE);
}

/**
 * @brief  Configure the TIM1 Pins.
 */
void TIM_GPIO_Configuration(void) {
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    // TIM1 CHx:PA8,PA9,PA10,PA11A     CHxN:PB13,PB14,PB15
    GPIO_InitStructure.Pin            = GPIO_PIN_8;  // | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}
