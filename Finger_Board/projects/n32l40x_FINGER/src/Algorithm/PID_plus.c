#include <math.h>

#include "PID_plus.h"

void PID_Init(PIDController* pid, float kp, float ki, float kd, float setpoint) {
    pid->kp                       = kp;
    pid->ki                       = ki;
    pid->kd                       = kd;
    pid->setpoint                 = setpoint;
    pid->integrator               = 0.0f;
    pid->prev_error               = 0.0f;
    pid->prev_measurement         = 0.0f;
    pid->integrator_limit         = 1.0e6f;
    pid->output_limit             = 1.0e6f;
    pid->dead_zone                = 0.0f;
    pid->rate_limit               = 1.0e6f;
    pid->ff_gain                  = 1.0f;
    pid->flags                    = 0;
    pid->filter_function          = NULL;
    pid->gain_scheduling_function = NULL;
    pid->fuzzy_function           = NULL;
}

float PID_Compute(PIDController* pid, float measurement) {
    float error   = pid->setpoint - measurement;
    float d_input = measurement - pid->prev_measurement;
    float output  = 0.0f;

    if (pid->flags & PID_DEAD_ZONE && fabs(error) < pid->dead_zone) {
        error = 0.0f;
    }

    if (pid->flags & PID_ANTI_WINDUP) {
        if (pid->integrator > pid->integrator_limit) {
            pid->integrator = pid->integrator_limit;
        } else if (pid->integrator < -pid->integrator_limit) {
            pid->integrator = -pid->integrator_limit;
        }
    }

    if (pid->flags & PID_FEEDFORWARD) {
        output += pid->ff_gain * pid->setpoint;
    }

    if (pid->flags & PID_INCREMENTAL) {
        output += pid->kp * (error - pid->prev_error);
        output += pid->ki * error;
        output -= pid->kd * d_input;
    } else {
        pid->integrator += pid->ki * error;
        output += pid->kp * error;
        output += pid->integrator;
        output -= pid->kd * d_input;
    }

    if (pid->flags & PID_FILTER && pid->filter_function != NULL) {
        output = pid->filter_function(output);
    }

    if (pid->flags & PID_GAIN_SCHEDULING && pid->gain_scheduling_function != NULL) {
        float new_kp, new_ki, new_kd;
        pid->gain_scheduling_function(pid->setpoint, measurement, &new_kp, &new_ki, &new_kd);
        pid->kp = new_kp;
        pid->ki = new_ki;
        pid->kd = new_kd;
    }

    if (pid->flags & PID_FUZZY && pid->fuzzy_function != NULL) {
        float new_kp, new_ki, new_kd;
        pid->fuzzy_function(error, d_input, &new_kp, &new_ki, &new_kd);
        pid->kp = new_kp;
        pid->ki = new_ki;
        pid->kd = new_kd;
    }

    if (pid->flags & PID_RATE_LIMIT) {
        float rate = (output - pid->prev_error) / pid->rate_limit;
        if (rate > pid->rate_limit) {
            output = pid->prev_error + pid->rate_limit;
        } else if (rate < -pid->rate_limit) {
            output = pid->prev_error - pid->rate_limit;
        }
    }

    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    pid->prev_error       = error;
    pid->prev_measurement = measurement;

    return output;
}

void PID_SetTunings(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetIntegratorLimit(PIDController* pid, float limit) { pid->integrator_limit = limit; }

void PID_SetOutputLimit(PIDController* pid, float limit) { pid->output_limit = limit; }

void PID_EnableFeature(PIDController* pid, unsigned int feature, int enable) {
    if (enable) {
        pid->flags |= feature;
    } else {
        pid->flags &= ~feature;
    }
}

void PID_SetFilterFunction(PIDController* pid, float (*filter_function)(float)) {
    pid->filter_function = filter_function;
}

void PID_SetGainSchedulingFunction(PIDController* pid,
                                   void (*gain_scheduling_function)(float, float, float*, float*, float*)) {
    pid->gain_scheduling_function = gain_scheduling_function;
}

void PID_SetFuzzyFunction(PIDController* pid, void (*fuzzy_function)(float, float, float*, float*, float*)) {
    pid->fuzzy_function = fuzzy_function;
}

// /* --------------------------------------- */
// #include <stdio.h>

// #include "pid_controller.h"

// // 示例滤波器函数
// float low_pass_filter(float input) {
//     static float prev_output = 0.0f;
//     float alpha              = 0.1f;
//     float output             = alpha * input + (1.0f - alpha) * prev_output;
//     prev_output              = output;
//     return output;
// }

// // 示例增益调度函数
// void gain_scheduling(float setpoint, float measurement, float* kp, float* ki, float* kd) {
//     // 简单的增益调度示例
//     if (measurement < setpoint) {
//         *kp = 1.0f;
//         *ki = 0.5f;
//         *kd = 0.1f;
//     } else {
//         *kp = 0.8f;
//         *ki = 0.4f;
//         *kd = 0.05f;
//     }
// }

// // 示例模糊控制函数
// void fuzzy_control(float error, float d_input, float* kp, float* ki, float* kd) {
//     // 简单的模糊控制示例
//     *kp = 1.0f + 0.1f * error;
//     *ki = 0.5f + 0.05f * error;
//     *kd = 0.1f + 0.01f * d_input;
// }

// int main() {
//     PIDController pid;
//     PID_Init(&pid, 1.0f, 0.1f, 0.01f, 100.0f);

//     PID_SetIntegratorLimit(&pid, 10.0f);
//     PID_SetOutputLimit(&pid, 100.0f);

//     PID_EnableFeature(&pid, PID_ANTI_WINDUP, 1);
//     PID_EnableFeature(&pid, PID_FEEDFORWARD, 1);
//     PID_EnableFeature(&pid, PID_INCREMENTAL, 0);
//     PID_EnableFeature(&pid, PID_FILTER, 1);
//     PID_SetFilterFunction(&pid, low_pass_filter);
//     PID_EnableFeature(&pid, PID_DEAD_ZONE, 1);
//     PID_EnableFeature(&pid, PID_RATE_LIMIT, 1);
//     PID_EnableFeature(&pid, PID_GAIN_SCHEDULING, 1);
//     PID_SetGainSchedulingFunction(&pid, gain_scheduling);
//     PID_EnableFeature(&pid, PID_FUZZY, 1);
//     PID_SetFuzzyFunction(&pid, fuzzy_control);

//     float measurement = 90.0f;
//     float output      = PID_Compute(&pid, measurement);
//     printf("PID Output: %f\n", output);

//     return 0;
// }
