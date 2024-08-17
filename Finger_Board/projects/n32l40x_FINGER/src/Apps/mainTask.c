/**
 * @file mainTask.c
 * @author Alexavier
 * @brief
 * @version 0.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "mainTask.h"

#include <stdint.h>

#include "N20.h"
#include "OPAMP.h"
#include "VOFA.h"
#include "clock.h"
#include "main.h"
#include "recvCmdTask.h"

/**
 * @brief
 *
 */
void taskLoop(void) {
    static uint8_t i = 0;
    uint8_t j        = 0;
    sysTimeUpdateIRQ();
			
		TIM_SetCmp3(TIM1, 2000);
	
    if (i % 2 == 0) {
        for (j = 0; j < N20_NUM; j++) {
            updateN20(&n20[j], getSysTime_s());
            n20Ctrl(&n20[j]);
        }
    }

    if (i % 5 == 0) {
        vofa_justfloat[FINGER_STATE]       = (float)0;
        vofa_justfloat[N20_SPD_ORDER_0]    = (float)n20[0].spd;
        vofa_justfloat[N20_SPD_ORDER_1]    = (float)n20[1].spd;
        vofa_justfloat[N20_POS_ORDER_0]    = (float)n20[0].pos;
        vofa_justfloat[N20_POS_ORDER_1]    = (float)n20[1].pos;
        vofa_justfloat[Gage_ORGER_1]       = (float)adc_value[0];
        vofa_justfloat[Gage_ORDER_2]       = (float)adc_value[1];
        vofa_justfloat[N20_OUTPUT_ORDER_0] = (float)n20[0].output;
        vofa_justfloat[N20_OUTPUT_ORDER_1] = (float)n20[1].output;
        vofa_justfloat[N20_SPD_TARGET_0]   = (float)n20[0].spd_tar;
        vofa_justfloat[N20_SPD_TARGET_1]   = (float)n20[1].spd_tar;
        vofaSend();
    }

    if (i % 10 == 0) {
        recvCmdTask(getSysTime_s());
    }

    i++;
    i %= 10;
}
