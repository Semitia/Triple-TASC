#ifndef __PID_H
#define __PID_H

typedef struct __PID_InitType {
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数
    float err;       // 误差
    float err_prev;  // 上次误差
    float err_sum;   // 误差累计
    float target;    // 目标值
    float temp;      // 当前值
    float I_limit;   // 积分限幅
    float res;       // PID输出
    float res_max;   // PID输出上限
    float res_min;   // PID输出下限
    float ts;        // 上次执行时间戳/s

    float ANTI_WINDUP_THRE;  // 抗积分饱和阈值
} PID_InitType;

typedef struct __PID_t {
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数
    float err;       // 误差
    float err_prev;  // 上次误差
    float err_sum;   // 误差累计
    float target;    // 目标值
    float temp;      // 当前值
    float I_limit;   // 积分限幅
    float res;       // PID输出
    float res_max;   // PID输出上限
    float res_min;   // PID输出下限
    float ts;        // 上次执行时间戳/s

    float ANTI_WINDUP_THRE;  // 抗积分饱和阈值

} PID_t;

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float I_lim, float res_max, float res_min, float ts,
              float ANTI_WINDUP_THRE);
float PID(PID_t *pid, float target, float temp, float ts);

#endif
