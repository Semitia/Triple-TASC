/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "main.h"

#include <stdint.h>
#include <stdio.h>

#include "LED.h"
#include "N20.h"
#include "OPAMP.h"
#include "TIM1_PWM.h"
#include "TIM2_ENC.h"
#include "TIM4_ENC.h"
#include "TIM6_CNT.h"
#include "TIM9_PWM.h"
#include "VOFA.h"
#include "clock.h"

/**
 * @brief  Main program.
 */
int main(void) {
    N20_InitType n20_init = {0};

    //timInit(TIM1);
    tim6Init();
    LedInit(GPIOB, GPIO_PIN_0);
		LedOff(GPIOB, GPIO_PIN_0); // DRV8835

    tim9Init();
    EncodeTime2Init();
    EncodeTime4Init();
    ns_opamp_init();

    vofaInit();
    n20_init.id          = 0;
    n20_init.reduc_ratio = REDUCTION_RATIO;
    n20_init.polar       = 1;
    n20_init.tim_enc     = TIM2;
    n20_init.tim_pwm     = TIM9;
    n20_init.setCmp_A    = TIM_SetCmp3;
    n20_init.setCmp_B    = TIM_SetCmp4;
    n20_init.GPIOx[0]    = GPIOB;
    n20_init.GPIOx[1]    = GPIOB;
    n20_init.GPIO_Pin[0] = GPIO_PIN_1;
    n20_init.GPIO_Pin[1] = GPIO_PIN_2;
    n20_init.ts          = 0;
    initN20(&n20[0], n20_init);
    n20_init.id          = 1;
    n20_init.reduc_ratio = REDUCTION_RATIO;
    n20_init.polar       = 1;
    n20_init.tim_enc     = TIM4;
    n20_init.tim_pwm     = TIM9;
    n20_init.setCmp_A    = TIM_SetCmp1;
    n20_init.setCmp_B    = TIM_SetCmp2;
    n20_init.GPIOx[0]    = GPIOB;
    n20_init.GPIOx[1]    = GPIOB;
    n20_init.GPIO_Pin[0] = GPIO_PIN_1;
    n20_init.GPIO_Pin[1] = GPIO_PIN_2;
    n20_init.ts          = 0;
    initN20(&n20[1], n20_init);

    TIM_Enable(TIM6, ENABLE);  // start main task
    while (1) {
    }
}

/**
 * @brief Assert failed function by user.
 * @param file The name of the call that failed.
 * @param line The source line number of the call that failed.
 */
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t *expr, const uint8_t *file, uint32_t line) {
    while (1) {
    }
}
#endif  // USE_FULL_ASSERT

/**
 * @}
 */
