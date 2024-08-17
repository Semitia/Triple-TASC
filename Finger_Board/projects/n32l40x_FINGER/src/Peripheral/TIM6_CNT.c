/**
 * @file TIM6_CNT.c
 * @author Alexavier
 * @brief use TIM6 as the clock, while generating interrupts every 1ms
 * @version 0.1
 * @date 2024-07-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "TIM6_CNT.h"

/**
 * @brief  Configures tim6 clocks.
 */
void tim6Init(void) {
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;

    /* PCLK1 = HCLK/4 */
    // RCC_ConfigPclk1(RCC_HCLK_DIV4);
    /* TIM6 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE);

    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Time base configuration */
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = 999;  // 1ms
    TIM_TimeBaseStructure.Prescaler = 31;   // 1M
    // TIM_TimeBaseStructure.Period    = 999; // 1s
    // TIM_TimeBaseStructure.Prescaler = 31999;  // 48M/48000 = 1k
    TIM_TimeBaseStructure.ClkDiv  = 0;
    TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM6, &TIM_TimeBaseStructure);

    /* TIM6 enable update irq */
    TIM_ConfigInt(TIM6, TIM_INT_UPDATE, ENABLE);
    /* TIM6 enable counter */
    // TIM_Enable(TIM6, ENABLE);
}
