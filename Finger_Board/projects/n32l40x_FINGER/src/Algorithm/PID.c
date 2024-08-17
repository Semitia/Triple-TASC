#include "PID.h"
#include "user_lib.h"
#include <math.h>

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))  // 限幅函数

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float I_lim, float res_max, float res_min, float ts,
              float ANTI_WINDUP_THRE) {
    pid->Kp               = Kp;
    pid->Ki               = Ki;
    pid->Kd               = Kd;
    pid->target           = 0;
    pid->temp             = 0;
    pid->err              = 0;
    pid->err_prev         = 0;
    pid->err_sum          = 0;
    pid->ts               = 0;
    pid->I_limit          = I_lim;
    pid->res_max          = res_max;
    pid->res_min          = res_min;
    pid->ANTI_WINDUP_THRE = ANTI_WINDUP_THRE;
}

float PID(PID_t *pid, float target, float temp, float ts) {
    float P, I, D;
    float dt = ts - pid->ts;

    // 抗积分饱和
    if (absf(target - pid->target) > pid->ANTI_WINDUP_THRE) {
        pid->err_sum = 0;
    }

    pid->target = target;
    pid->temp   = temp;
    pid->err    = target - temp;
    P           = pid->Kp * pid->err;
    pid->err_sum += pid->err * dt;
    pid->err_sum = _constrain(pid->err_sum, -pid->I_limit, pid->I_limit);
    I            = pid->Ki * pid->err_sum;
    D            = pid->Kd * (pid->err - pid->err_prev) / dt;

    pid->res = P + I + D;
    pid->res = _constrain(pid->res, pid->res_min, pid->res_max);
    // update
    pid->err_prev = pid->err;
    pid->ts       = ts;
    return pid->res;
}
