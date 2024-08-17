#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integrator;
    float prev_error;
    float prev_measurement;
    float integrator_limit;
    float output_limit;
    float dead_zone;
    float rate_limit;
    float ff_gain;
    unsigned int flags;
    float (*filter_function)(float);
    void (*gain_scheduling_function)(float, float, float*, float*, float*);
    void (*fuzzy_function)(float, float, float*, float*, float*);
} PIDController;

#define PID_ANTI_WINDUP (1 << 0)
#define PID_FEEDFORWARD (1 << 1)
#define PID_INCREMENTAL (1 << 2)
#define PID_FILTER (1 << 3)
#define PID_DEAD_ZONE (1 << 4)
#define PID_RATE_LIMIT (1 << 5)
#define PID_GAIN_SCHEDULING (1 << 6)
#define PID_FUZZY (1 << 7)

void PID_Init(PIDController* pid, float kp, float ki, float kd, float setpoint);
float PID_Compute(PIDController* pid, float measurement);
void PID_SetTunings(PIDController* pid, float kp, float ki, float kd);
void PID_SetIntegratorLimit(PIDController* pid, float limit);
void PID_SetOutputLimit(PIDController* pid, float limit);
void PID_EnableFeature(PIDController* pid, unsigned int feature, int enable);
void PID_SetFilterFunction(PIDController* pid, float (*filter_function)(float));
void PID_SetGainSchedulingFunction(PIDController* pid,
                                   void (*gain_scheduling_function)(float, float, float*, float*, float*));
void PID_SetFuzzyFunction(PIDController* pid, void (*fuzzy_function)(float, float, float*, float*, float*));

#endif
