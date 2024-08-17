/**
 * @file TIM2_ENC.c
 * @author Alexavier
 * @brief use TIM2 to capture encoder signal
 * @version 0.1
 * @date 2024-07-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "TIM2_ENC.h"

#include "n32l40x.h"

void EncodeTime2Init(void) {
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_ICInitType TIM_ICInitStructure;
    NVIC_InitType NVIC_InitStructure;
    GPIO_InitType GPIO_InitStructure;

    /* clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    /* GPIO Initial */
    GPIO_InitStruct(&GPIO_InitStructure);
    /*PA0 - Encode Signal Input port1*/
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;      // GPIO_DC_LOW;  //
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;  // GPIO_MODE_AF_PP; //
    GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;     // GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM2;
    GPIO_InitStructure.Pin            = GPIO_PIN_0;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    /*PA1 - Encode Signal Input port2*/
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM2;
    GPIO_InitStructure.Pin            = GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_DeInit(TIM2);
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM2, &TIM_TimeBaseStructure);

    TIM_ConfigEncoderInterface(TIM2, TIM_ENCODE_MODE_TI12, TIM_IC_POLARITY_RISING, TIM_IC_POLARITY_RISING);
    TIM_InitIcStruct(&TIM_ICInitStructure);
    TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcFilter   = 6;
    TIM_ICInitStructure.Channel    = TIM_CH_1;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.Channel    = TIM_CH_2;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ConfigArPreload(TIM2, ENABLE);
    TIM_SetCnt(TIM2, 0);
    TIM_Enable(TIM2, ENABLE);

    TIM_ClearFlag(TIM2, TIM_FLAG_UPDATE);
    // TIM_ConfigInt(TIM2, TIM_INT_UPDATE, ENABLE); // 重装载中断
}

// /*获取电机转动圈数*/
// void TIM2_IRQHandler(void) {
//     if (TIM_GetIntStatus(TIM2, TIM_INT_UPDATE) != RESET) {
//         TIM_ClrIntPendingBit(TIM2, TIM_INT_UPDATE);
//     }
// }
