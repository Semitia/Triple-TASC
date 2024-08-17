/**
 * @file TIM2_ENC.h
 * @author Alexavier
 * @brief use TIM2 to capture encoder signal
 * @version 0.1
 * @date 2024-07-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __TIM2_ENC_H__
#define __TIM2_ENC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_lib.h"

#define ENCODER_LINE_CNT (7)        // 编码器线数，根据实际编码器参数设定
#define ENCODER_TIM_PERIOD (65535)  // 解码定时器计数周期,模式3下计数会为4倍，模式1、2下计数为两倍

void EncodeTime2Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM2_ENC_H__ */
