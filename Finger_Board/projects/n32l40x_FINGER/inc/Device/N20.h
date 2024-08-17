/**
 * @file N20.h
 * @author Alexavier
 * @brief N20 control lib for N32G030
 * @version 0.1
 * @date 2024-07-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __N20_H__
#define __N20_H__

#include <stdint.h>

#include "PID.h"
#include "filters.h"
#include "n32l40x.h"

#define N20_NUM              2
#define LP_K                 0.02
#define ENCODER_PPR          7     // 编码器基础脉冲数
#define REDUCTION_RATIO      120   // 减速比
#define REDUCTION_RATIO_WORM 236   // 蜗杆减速器减速比
#define REDUCTION_RATIO_GEAR 380   // 直齿轮箱
#define SPD_SEND_SCALE       1000  // 电机转速发送缩放比例
#define POS_SEND_SCALE       1000  // 电机位置发送缩放比例

// #define TB6612
#define DRV8833

enum ctrl_mode { SPD_CTRL = 0, POS_CTRL, SHUT_DOWN };

typedef struct __N20_InitType {
    uint8_t id;
    float reduc_ratio;
    short polar;
    TIM_Module *tim_enc;
    TIM_Module *tim_pwm;
    void (*setCmp_A)(TIM_Module *TIMx, uint16_t Compare1);  // 设置PWM占空比
    void (*setCmp_B)(TIM_Module *TIMx, uint16_t Compare2);  // 设置PWM占空比
    GPIO_Module *GPIOx[2];                                  // 电机控制引脚
    uint16_t GPIO_Pin[2];                                   // 电机控制引脚
    float ts;                                               // 时间戳/s
} N20_InitType;

typedef struct __N20_t {
    uint8_t id;
    enum ctrl_mode mode;

    // Encoder TIM
    int16_t encoder;
    TIM_Module *TIM_ENC;
    float last_time;  // second

    // PWM TIM
    TIM_Module *TIM_PWM;
    uint16_t pwm_arr;                                       // PWM TIM 自动重装值
    void (*setCmp_A)(TIM_Module *TIMx, uint16_t Compare1);  // 设置PWM占空比
    void (*setCmp_B)(TIM_Module *TIMx, uint16_t Compare2);  // 设置PWM占空比
    GPIO_Module *GPIOx[2];                                  // 电机控制引脚
    uint16_t GPIO_Pin[2];                                   // 电机控制引脚

    float output;  // 电机输出 0~1
    short output_polar;
    float enc_spd_ratio;           // 电机转速与编码器转速比
    float spd, spd_tar, spd_last;  // rad/s
    float pos, pos_tar, pos_nor;   // position, target, normalized  rad

    PID_t SpdPID, PosPID;
    LowPass_t SpdLP;
} N20_t;

extern N20_t n20[2];

void setPWM(N20_t *n20);
void spdCtrl(N20_t *n20);
void posCtrl(N20_t *n20);
void n20Ctrl(N20_t *n20);
void shutDown(N20_t *n20);
void updateN20(N20_t *n20, float ts);
void setSpd(N20_t *n20, float spd_target);
void setPos(N20_t *n20, float pos_target);
void initN20(N20_t *n20, N20_InitType initParams);

/*--------------------------- middleware ----------------------------*/

#endif  // __N20_H__
