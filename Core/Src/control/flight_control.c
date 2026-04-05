#include "control/flight_control.h"

#include "platform/tim.h"
#include "sensor/sensor_common.h"

#include <math.h>

#define MIN_ARM 1240

static MPC_Status_t last_MPC_Status = HOVER;
static uint8_t reset_pid_request = 0;

static void MIX_THROTTLE(float32_t thr, float32_t *moment, float32_t *m)
{
    int i;

    m[0] = thr - moment[0] + moment[1] + moment[2];
    m[1] = thr - moment[0] - moment[1] - moment[2];
    m[2] = thr + moment[0] - moment[1] + moment[2];
    m[3] = thr + moment[0] + moment[1] - moment[2];

    for (i = 0; i < 4; i++) {
        if (m[i] > 1850) {
            m[i] = 1850;
        }
        if (m[i] < MIN_ARM) {
            m[i] = MIN_ARM;
        }
    }
}

static void Control_Motor(void)
{
    int i;

    if (enable_motor) {
        for (i = 0; i < 4; i++) {
            PWM_TIMER[i] = (uint32_t)PWM_MOTOR[i];
        }
        TIM3->CCR1 = PWM_TIMER[0];
        TIM3->CCR2 = PWM_TIMER[1];
        TIM4->CCR1 = PWM_TIMER[2];
        TIM4->CCR2 = PWM_TIMER[3];
    } else {
        TIM3->CCR1 = 1000;
        TIM3->CCR2 = 1000;
        TIM4->CCR1 = 1000;
        TIM4->CCR2 = 1000;
    }
}

void FlightController_InitMotorOutputs(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    TIM3->CCR1 = 1000;
    TIM3->CCR2 = 1000;
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
}

void RESET_ALL_PID(void)
{
    Reset_PID_ALTIDUE(&PID_RATE_ROLL);
    Reset_PID_ALTIDUE(&PID_RATE_PITCH);
    Reset_PID_ALTIDUE(&PID_RATE_YAW);
    Reset_PID_ALTIDUE(&PID_ROLL);
    Reset_PID_ALTIDUE(&PID_PITCH);
    Reset_PID_ALTIDUE(&PID_YAW);
}

void MPC(void)
{
    float32_t feedback[3];
    float32_t real_dt = MPU6500_DATA.dt;

    if (real_dt > 0.01f) {
        real_dt = 0.01f;
    }
    if (real_dt < 0.001f) {
        real_dt = 0.001f;
    }

    if (RC_Raw_Throttle < 950) {
        if (ARM_Status == ARM) {
            MPC_Status = HOVER;
            angle_desired[0] = 0.0f;
            angle_desired[1] = 0.0f;
            Throttle -= 0.2f;
            if (Throttle < 1100.0f) {
                ARM_Status = NOT_ARM;
                enable_motor = 0;
                Throttle = 1000.0f;
            }
        }
    } else {
        float32_t stick_roll;
        float32_t stick_pitch;
        float32_t stick_yaw;

        if (RC_Raw_Throttle > 2000) {
            Throttle = 2000.0f;
        } else if (RC_Raw_Throttle < 1000) {
            Throttle = 1000.0f;
        } else {
            Throttle = (float32_t)RC_Raw_Throttle;
        }

        {
            MPC_Status_t current_Mode = (RC_Raw_SW_Mode > 1500) ? HOVER : RATE_MODE;
            if (current_Mode != last_MPC_Status) {
                MPC_Status = current_Mode;
                last_MPC_Status = current_Mode;
                reset_pid_request = 1;
            }
        }

        if (RC_Raw_SW_Arm > 1500) {
            if (ARM_Status == NOT_ARM && Throttle < 1150) {
                ARM_Status = ARM;
                enable_motor = 1;
                RESET_ALL_PID();
                angle_desired[0] = 0;
                angle_desired[1] = 0;
                angle_desired[2] = Complimentary_Filter.Euler_Angle_Deg[2];
                reset_pid_request = 1;
            }
        } else {
            ARM_Status = NOT_ARM;
            enable_motor = 0;
        }

        stick_roll = (float32_t)RC_Raw_Roll - 1500.0f;
        stick_pitch = (float32_t)RC_Raw_Pitch - 1500.0f;
        stick_yaw = (float32_t)RC_Raw_Yaw - 1500.0f;

        if (fabsf(stick_yaw) < 15.0f) {
            stick_yaw = 0.0f;
        }
        if (fabsf(stick_roll) < 5.0f) {
            stick_roll = 0.0f;
        }
        if (fabsf(stick_pitch) < 5.0f) {
            stick_pitch = 0.0f;
        }

        if (MPC_Status == HOVER) {
            float yaw_speed = 150.0f;
            float angle_step;

            angle_desired[0] = stick_roll * 0.06f;
            angle_desired[1] = -stick_pitch * 0.06f;

            angle_step = (stick_yaw / 500.0f) * yaw_speed * real_dt;
            angle_desired[2] += angle_step;

            if (angle_desired[2] > 180.0f) {
                angle_desired[2] -= 360.0f;
            }
            if (angle_desired[2] < -180.0f) {
                angle_desired[2] += 360.0f;
            }
        } else {
            angle_rate_desired[0] = stick_roll * 0.20f;
            angle_rate_desired[1] = -stick_pitch * 0.20f;
            angle_rate_desired[2] = stick_yaw * 0.20f;
        }
    }

    if (ARM_Status == ARM) {
        if (Throttle < 1500.0f) {
            PID_ROLL.integral = 0;
            PID_PITCH.integral = 0;
            PID_YAW.integral = 0;
            PID_RATE_ROLL.integral = 0;
            PID_RATE_PITCH.integral = 0;
            PID_RATE_YAW.integral = 0;
        }

        switch (MPC_Status) {
        case RATE_MODE:
            if (reset_pid_request) {
                PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                reset_pid_request = 0;
            }

            feedback[0] = MPU6500_DATA.w[0] * RAD_TO_DEG;
            feedback[1] = MPU6500_DATA.w[1] * RAD_TO_DEG;
            feedback[2] = MPU6500_DATA.w[2] * RAD_TO_DEG;

            Caculate_PID_Rate_ALTIDUE(&PID_RATE_ROLL, angle_rate_desired[0], feedback[0], real_dt);
            Caculate_PID_Rate_ALTIDUE(&PID_RATE_PITCH, angle_rate_desired[1], feedback[1], real_dt);
            Caculate_PID_Rate_ALTIDUE(&PID_RATE_YAW, angle_rate_desired[2], feedback[2], real_dt);

            Moment[0] = PID_RATE_ROLL.output;
            Moment[1] = PID_RATE_PITCH.output;
            Moment[2] = PID_RATE_YAW.output;

            MIX_THROTTLE(Throttle, Moment, PWM_MOTOR);
            Control_Motor();
            break;

        case HOVER:
            feedback[0] = Complimentary_Filter.Euler_Angle_Deg[0];
            feedback[1] = Complimentary_Filter.Euler_Angle_Deg[1];
            feedback[2] = Complimentary_Filter.Euler_Angle_Deg[2];

            Caculate_PID_ALTIDUE(&PID_ROLL, angle_desired[0], feedback[0], real_dt);
            Caculate_PID_ALTIDUE(&PID_PITCH, angle_desired[1], feedback[1], real_dt);
            Caculate_PID_ALTIDUE(&PID_YAW, angle_desired[2], feedback[2], real_dt);

            angle_rate_desired[0] = PID_ROLL.output;
            angle_rate_desired[1] = PID_PITCH.output;
            angle_rate_desired[2] = PID_YAW.output;

            if (reset_pid_request) {
                PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                reset_pid_request = 0;
            }

            feedback[0] = MPU6500_DATA.w[0] * RAD_TO_DEG;
            feedback[1] = MPU6500_DATA.w[1] * RAD_TO_DEG;
            feedback[2] = MPU6500_DATA.w[2] * RAD_TO_DEG;

            Caculate_PID_Rate_ALTIDUE(&PID_RATE_ROLL, angle_rate_desired[0], feedback[0], real_dt);
            Caculate_PID_Rate_ALTIDUE(&PID_RATE_PITCH, angle_rate_desired[1], feedback[1], real_dt);
            Caculate_PID_Rate_ALTIDUE(&PID_RATE_YAW, angle_rate_desired[2], feedback[2], real_dt);

            Moment[0] = PID_RATE_ROLL.output;
            Moment[1] = PID_RATE_PITCH.output;
            Moment[2] = PID_RATE_YAW.output;

            MIX_THROTTLE(Throttle, Moment, PWM_MOTOR);
            Control_Motor();
            break;
        }
    } else {
        int i;

        for (i = 0; i < 4; i++) {
            PWM_MOTOR[i] = 1000;
            PWM_TIMER[i] = 1000;
        }
        Control_Motor();
        RESET_ALL_PID();

        angle_rate_desired[0] = 0;
        angle_rate_desired[1] = 0;
        angle_rate_desired[2] = 0;
        angle_desired[0] = 0;
        angle_desired[1] = 0;
        angle_desired[2] = Complimentary_Filter.Euler_Angle_Deg[2];
    }
}
