/**
 * @file N20.c
 * @brief N20电机STM32驱动
 * @author Semitia
 * @date 2023-12-12
 * @version 0.1
 */

#include "N20.h"

#include "user_lib.h"

N20_t n20[N20_NUM] = {0};

void initN20(N20_t *n20, N20_InitType initParams) {
    n20->id            = initParams.id;
    n20->mode          = SPD_CTRL;
    n20->enc_spd_ratio = (float)2 * PI / (ENCODER_PPR * initParams.reduc_ratio * 4);  // 双相编码器，4倍频
    n20->TIM_ENC       = initParams.tim_enc;
    n20->TIM_PWM       = initParams.tim_pwm;
    n20->GPIOx[0]      = initParams.GPIOx[0];
    n20->GPIOx[1]      = initParams.GPIOx[1];
    n20->GPIO_Pin[0]   = initParams.GPIO_Pin[0];
    n20->GPIO_Pin[1]   = initParams.GPIO_Pin[1];
    n20->output        = 0;
    n20->output_polar  = initParams.polar;
    n20->last_time     = 0;
    n20->pwm_arr       = TIM_GetAutoReload(n20->TIM_PWM);
    n20->setCmp_A      = initParams.setCmp_A;
    n20->setCmp_B      = initParams.setCmp_B;

    n20->spd      = 0;
    n20->spd_tar  = 0;
    n20->spd_last = 0;
    n20->pos      = 0;
    n20->pos_tar  = 0;
    n20->pos_nor  = 0;

    lowPassInit(&n20->SpdLP, LP_K);
    n20->SpdPID.Kp               = 1;
    n20->SpdPID.Ki               = 1;
    n20->SpdPID.Kd               = 0.03;
    n20->SpdPID.I_limit          = 0.3;
    n20->SpdPID.res_max          = 1.0;
    n20->SpdPID.res_min          = -1.0;
    n20->SpdPID.ts               = initParams.ts;
    n20->SpdPID.ANTI_WINDUP_THRE = 1.5f;

    n20->PosPID.Kp      = 3;
    n20->PosPID.Ki      = 0.0;
    n20->PosPID.Kd      = 0.2;
    n20->PosPID.I_limit = 6.0;
    n20->PosPID.res_max = 4.0;
    n20->PosPID.res_min = -4.0;
    n20->PosPID.ts      = initParams.ts;
}

/**
 * @brief 更新电机状态
 * @param n20 N20电机结构体
 */
void updateN20(N20_t *n20, float ts) {
    float dx, dt;
    int16_t cnt = (int16_t)TIM_GetCnt(n20->TIM_ENC);

    dx           = (float)(cnt * n20->enc_spd_ratio);
    dt           = (float)(ts - n20->last_time);
    n20->encoder = cnt;
    n20->pos += dx;
    n20->spd_last = n20->spd;
    n20->spd      = lowPass(&n20->SpdLP, (dx / dt));
    TIM_SetCnt(n20->TIM_ENC, 0);  // reset count
    n20->last_time = ts;
    return;
}

void spdCtrl(N20_t *n20) {
    n20->output = PID(&n20->SpdPID, n20->spd_tar, n20->spd, n20->last_time) * n20->output_polar;
    return;
}

void posCtrl(N20_t *n20) {
    n20->spd_tar = PID(&n20->PosPID, n20->pos_tar, n20->pos, n20->last_time);
    spdCtrl(n20);
    return;
}

void setSpd(N20_t *n20, float spd_target) {
    n20->spd_tar = spd_target;
    n20->mode    = SPD_CTRL;
    return;
}

void setPos(N20_t *n20, float pos_target) {
    n20->pos_tar = pos_target;
    n20->mode    = POS_CTRL;
    return;
}

void shutDown(N20_t *n20) {
    n20->output = 0;
    n20->mode   = SHUT_DOWN;
    return;
}

void n20Ctrl(N20_t *n20) {
    switch (n20->mode) {
        case SPD_CTRL:
            spdCtrl(n20);
            break;
        case POS_CTRL:
            posCtrl(n20);
            break;
        case SHUT_DOWN:
            shutDown(n20);
            break;
        default:
            break;
    }
    setPWM(n20);
    return;
}

/**
 * @brief 根据output设置电机PWM
 * @param n20 N20电机结构体
 */
#ifdef TB6612
void setPWM(N20_t *n20) {
    uint16_t CCR;
    if (n20->output >= 0) {
        CCR = (uint16_t)(n20->pwm_arr * n20->output);
        n20->setCmp_A(n20->TIM_PWM, CCR);
        GPIO_SetBits(n20->GPIOx[0], n20->GPIO_Pin[0]);
        GPIO_ResetBits(n20->GPIOx[1], n20->GPIO_Pin[1]);
    } else {
        CCR = (uint16_t)(n20->pwm_arr * (-n20->output));
        n20->setCmp_A(n20->TIM_PWM, CCR);
        GPIO_SetBits(n20->GPIOx[1], n20->GPIO_Pin[1]);
        GPIO_ResetBits(n20->GPIOx[0], n20->GPIO_Pin[0]);
    }
    return;
}

#elif defined(DRV8833)
void setPWM(N20_t *n20) {
    uint16_t CCR;
    if (n20->output >= 0) {
        CCR = (uint16_t)(n20->pwm_arr * n20->output);
        n20->setCmp_A(n20->TIM_PWM, CCR);
        n20->setCmp_B(n20->TIM_PWM, 0);
    } else {
        CCR = (uint16_t)(n20->pwm_arr * (-n20->output));
        n20->setCmp_A(n20->TIM_PWM, 0);
        n20->setCmp_B(n20->TIM_PWM, CCR);
    }
    return;
}

#endif
