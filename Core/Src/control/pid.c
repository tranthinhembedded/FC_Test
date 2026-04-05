#include "control/pid.h"

#include <math.h>

static float32_t clamp_value(float32_t value, float32_t min, float32_t max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

void Reset_PID_ALTIDUE(PID_ALTIDUE_t *pid)
{
    pid->derivative = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->prev_measure = 0;
    pid->prev_setpoint = 0;
}

void Caculate_PID_ALTIDUE(PID_ALTIDUE_t *pid, float32_t setpoint, float32_t feedback, float32_t dt)
{
    if (dt < 0.0001f) {
        dt = 0.0001f;
    }
    pid->error = setpoint - feedback;

    if (pid->error > 180.0f) {
        pid->error -= 360.0f;
    } else if (pid->error < -180.0f) {
        pid->error += 360.0f;
    }

    {
        float32_t P_term = pid->kp * pid->error;

        if (fabsf(pid->error) < 20.0f) {
            pid->integral += pid->ki * pid->error * dt;
            pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);
        }

        pid->output = P_term + pid->integral;
        pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);
    }

    pid->prev_error = pid->error;
    pid->prev_measure = feedback;
    pid->prev_setpoint = setpoint;
}

void Caculate_PID_Rate_ALTIDUE(PID_ALTIDUE_t *pid, float32_t setpoint, float32_t feedback, float32_t dt)
{
    if (dt < 0.0001f) {
        dt = 0.0001f;
    }
    pid->error = setpoint - feedback;

    {
        float32_t P_term = pid->kp * pid->error;
        float32_t delta_measure;
        float32_t D_raw;
        float32_t delta_setpoint;
        float32_t FF_term;
        float32_t ff_limit;

        pid->integral += pid->ki * pid->error * dt;
        pid->integral = clamp_value(pid->integral, -pid->i_limit, pid->i_limit);

        delta_measure = feedback - pid->prev_measure;
        D_raw = -(delta_measure / dt) * pid->kd;

        pid->derivative = pid->alpha_lpf * pid->derivative + (1.0f - pid->alpha_lpf) * D_raw;
        pid->derivative = clamp_value(pid->derivative, -pid->d_limit, pid->d_limit);

        delta_setpoint = setpoint - pid->prev_setpoint;
        FF_term = (delta_setpoint / dt) * pid->feed_forward;

        ff_limit = pid->max_output * 0.2f;
        FF_term = clamp_value(FF_term, -ff_limit, ff_limit);

        pid->output = P_term + pid->integral + pid->derivative + FF_term;
        pid->output = clamp_value(pid->output, -pid->max_output, pid->max_output);
    }

    pid->prev_error = pid->error;
    pid->prev_measure = feedback;
    pid->prev_setpoint = setpoint;
}
