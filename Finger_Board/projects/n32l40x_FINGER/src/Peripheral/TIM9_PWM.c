/**
 * @file TIM9_PWM.c
 * @author Alexavier
 * @brief use tim9 to generate PWM signal
 * @version 0.1
 * @date 2024-07-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "TIM9_PWM.h"

/**
 * @brief   intial timx program
 */
void tim9Init(void) {
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    GPIO_InitType GPIO_InitStructure;

    /* tim9, GPIOB and AFIO clocks enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM9, ENABLE);

    /* GPIO Configuration */
    GPIO_InitStruct(&GPIO_InitStructure);
    // tim9 CHx:PB12, PB13, PB14, PB15
    GPIO_InitStructure.Pin            = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_TIM9;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* Time Base configuration */
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Prescaler = 7;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.Period    = 999;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_InitTimeBase(TIM9, &TIM_TimeBaseStructure);

    /* Channel Configuration in PWM mode */
    TIM_InitOcStruct(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 500;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    // TIM_OCInitStructure.OcIdleState = TIM_OC_IDLE_STATE_SET;  // important for tim1
    TIM_InitOc1(TIM9, &TIM_OCInitStructure);
    TIM_OCInitStructure.Pulse = 500;
    TIM_InitOc2(TIM9, &TIM_OCInitStructure);
    TIM_OCInitStructure.Pulse = 500;
    TIM_InitOc3(TIM9, &TIM_OCInitStructure);
    TIM_OCInitStructure.Pulse = 500;
    TIM_InitOc4(TIM9, &TIM_OCInitStructure);

    TIM_ConfigOc1Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc2Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc3Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);
    TIM_ConfigOc4Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);

    TIM_ConfigArPreload(TIM9, ENABLE);
    TIM_Enable(TIM9, ENABLE); /* tim9 counter enable */
}

// uint16_t CCR1_Val       = 333;
// uint16_t CCR2_Val       = 249;
// uint16_t CCR3_Val       = 166;
// uint16_t CCR4_Val       = 83;
// uint16_t PrescalerValue = 0;
// void tim9Init(void) {
//     TIM_TimeBaseInitType TIM_TimeBaseStructure;
//     OCInitType TIM_OCInitStructure;
//     GPIO_InitType GPIO_InitStructure;

//    /* TIM3 clock enable */
//    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3 | RCC_APB1_PERIPH_TIM9, ENABLE);
//    /* GPIOA and GPIOB clock enable */
//    RCC_EnableAPB2PeriphClk(
//        RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_AFIO, ENABLE);

//    GPIO_InitStruct(&GPIO_InitStructure);
//    /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function push-pull */
//    GPIO_InitStructure.Pin            = GPIO_PIN_6 | GPIO_PIN_7;
//    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
//    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
//    // GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

//    GPIO_InitStructure.Pin            = GPIO_PIN_0 | GPIO_PIN_1;
//    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
//    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

//    GPIO_InitStructure.Pin            = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
//    GPIO_InitStructure.GPIO_Alternate = GPIO_AF1_TIM9;
//    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

//    /* Compute the prescaler value */
//    PrescalerValue = (uint16_t)(SystemCoreClock / 12000000) - 1;
//    /* Time base configuration */
//    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
//    TIM_TimeBaseStructure.Period    = 665;
//    TIM_TimeBaseStructure.Prescaler = PrescalerValue;
//    TIM_TimeBaseStructure.ClkDiv    = 0;
//    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

//    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
//    TIM_InitTimeBase(TIM9, &TIM_TimeBaseStructure);

//    /* PWM1 Mode configuration: Channel1 */
//    TIM_InitOcStruct(&TIM_OCInitStructure);
//    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
//    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
//    TIM_OCInitStructure.Pulse       = CCR1_Val;
//    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;

//    TIM_InitOc1(TIM3, &TIM_OCInitStructure);
//    TIM_InitOc1(TIM9, &TIM_OCInitStructure);

//    TIM_ConfigOc1Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
//    TIM_ConfigOc1Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);

//    /* PWM1 Mode configuration: Channel2 */
//    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
//    TIM_OCInitStructure.Pulse       = CCR2_Val;

//    TIM_InitOc2(TIM3, &TIM_OCInitStructure);
//    TIM_InitOc2(TIM9, &TIM_OCInitStructure);

//    TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
//    TIM_ConfigOc2Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);

//    /* PWM1 Mode configuration: Channel3 */
//    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
//    TIM_OCInitStructure.Pulse       = CCR3_Val;

//    TIM_InitOc3(TIM3, &TIM_OCInitStructure);
//    TIM_InitOc3(TIM9, &TIM_OCInitStructure);

//    TIM_ConfigOc3Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
//    TIM_ConfigOc3Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);

//    /* PWM1 Mode configuration: Channel4 */
//    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
//    TIM_OCInitStructure.Pulse       = CCR4_Val;

//    TIM_InitOc4(TIM3, &TIM_OCInitStructure);
//    TIM_InitOc4(TIM9, &TIM_OCInitStructure);

//    TIM_ConfigOc4Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
//    TIM_ConfigOc4Preload(TIM9, TIM_OC_PRE_LOAD_ENABLE);

//    TIM_ConfigArPreload(TIM3, ENABLE);
//    TIM_ConfigArPreload(TIM9, ENABLE);

//    /* TIM3 enable counter */
//    TIM_Enable(TIM3, ENABLE);
//    TIM_Enable(TIM9, ENABLE);
//}
