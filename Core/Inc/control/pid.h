#ifndef CORE_APP_CONTROL_PID_PID_H_
#define CORE_APP_CONTROL_PID_PID_H_

#include "arm_math.h"

typedef struct {
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t feed_forward;

    float32_t error;
    float32_t prev_error;

    float32_t integral;
    float32_t derivative;

    float32_t prev_measure;
    float32_t prev_setpoint;

    float32_t alpha_lpf;
    float32_t max_output;
    float32_t i_limit;
    float32_t d_limit;
    float32_t output;
} PID_ALTIDUE_t;

void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid);
void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid, float32_t setpoint, float32_t feedback, float32_t dt);
void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid, float32_t setpoint, float32_t feedback, float32_t dt);

#endif /* CORE_APP_CONTROL_PID_PID_H_ */
