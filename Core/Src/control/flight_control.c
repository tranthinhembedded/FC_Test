#include "control/flight_control.h"

#include "input/rc_input.h"
#include "platform/tim.h"
#include "sensor/sensor_common.h"
#include "sensor/sensor_check.h"

#include <math.h>

#define MIN_ARM 1100
#define MANUAL_TILT_SCALE_DEG               0.06f
#define MANUAL_RATE_SCALE_DPS               0.20f
#define MANUAL_YAW_SPEED_DPS                150.0f
#define MAX_ANGLE_TARGET_DEG                25.0f
#define OPTFLOW_STALE_TIMEOUT_S             0.15f
#define OPTFLOW_MIN_QUALITY                 0.25f
#define OPTFLOW_MIN_ALTITUDE_M              0.05f
#define OPTFLOW_MAX_ALTITUDE_M              10.0f
#define OPTFLOW_MAX_BODY_VELOCITY_MPS       4.0f
#define OPTFLOW_MAX_POSITION_ERROR_M        1.5f
#define OPTFLOW_CAPTURE_STICK_THRESHOLD_US  20.0f
#define OPTFLOW_RELEASE_STICK_THRESHOLD_US  35.0f
#define OPTFLOW_VELOCITY_LPF_GAIN           0.35f

static FlightController_OpticalFlowState_t optical_flow_state = {0};
static uint8_t optical_flow_lpf_inited = 0U;
static PID_ALTIDUE_t PID_POSHOLD_X = {
    .alpha_lpf = 0.85f, .feed_forward = 0.0f, .i_limit = 0.35f, .max_output = 0.8f,
    .kp = 1.10f, .ki = 0.08f, .kd = 0.0f, .d_limit = 0.25f,
};
static PID_ALTIDUE_t PID_POSHOLD_Y = {
    .alpha_lpf = 0.85f, .feed_forward = 0.0f, .i_limit = 0.35f, .max_output = 0.8f,
    .kp = 1.10f, .ki = 0.08f, .kd = 0.0f, .d_limit = 0.25f,
};
static PID_ALTIDUE_t PID_VELHOLD_X = {
    .alpha_lpf = 0.82f, .feed_forward = 0.0f, .i_limit = 3.5f, .max_output = 8.0f,
    .kp = 5.20f, .ki = 1.15f, .kd = 0.04f, .d_limit = 2.0f,
};
static PID_ALTIDUE_t PID_VELHOLD_Y = {
    .alpha_lpf = 0.82f, .feed_forward = 0.0f, .i_limit = 3.5f, .max_output = 8.0f,
    .kp = 5.20f, .ki = 1.15f, .kd = 0.04f, .d_limit = 2.0f,
};

static MPC_Status_t last_MPC_Status = HOVER;
static uint8_t reset_pid_request = 0;

static float32_t clamp_float(float32_t value, float32_t min_value, float32_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float32_t normalize_optical_flow_quality(float32_t quality)
{
    if (quality > 1.5f) {
        quality *= (1.0f / 255.0f);
    }
    return clamp_float(quality, 0.0f, 1.0f);
}

static void reset_optical_flow_controller(void)
{
    Reset_PID_ALTIDUE(&PID_POSHOLD_X);
    Reset_PID_ALTIDUE(&PID_POSHOLD_Y);
    Reset_PID_ALTIDUE(&PID_VELHOLD_X);
    Reset_PID_ALTIDUE(&PID_VELHOLD_Y);

    optical_flow_state.correction_deg[0] = 0.0f;
    optical_flow_state.correction_deg[1] = 0.0f;
    optical_flow_state.hold_locked = 0U;
}

static void invalidate_optical_flow_hold(void)
{
    optical_flow_state.healthy = 0U;
    optical_flow_state.data_age_s = OPTFLOW_STALE_TIMEOUT_S;
    reset_optical_flow_controller();
}

static void capture_optical_flow_hold_position(void)
{
    optical_flow_state.hold_position_earth_m[0] = optical_flow_state.position_earth_m[0];
    optical_flow_state.hold_position_earth_m[1] = optical_flow_state.position_earth_m[1];
    optical_flow_state.hold_locked = 1U;

    Reset_PID_ALTIDUE(&PID_POSHOLD_X);
    Reset_PID_ALTIDUE(&PID_POSHOLD_Y);
    Reset_PID_ALTIDUE(&PID_VELHOLD_X);
    Reset_PID_ALTIDUE(&PID_VELHOLD_Y);
}

static void age_optical_flow_data(float32_t dt)
{
    if (dt <= 0.0f) {
        return;
    }

    if (optical_flow_state.data_age_s < 10.0f) {
        optical_flow_state.data_age_s += dt;
    }

    if (optical_flow_state.data_age_s > OPTFLOW_STALE_TIMEOUT_S) {
        invalidate_optical_flow_hold();
    }
}

static void compute_optical_flow_correction(float32_t dt)
{
    float32_t yaw_rad;
    float32_t cos_yaw;
    float32_t sin_yaw;
    float32_t position_error_forward;
    float32_t position_error_right;
    float32_t tilt_earth_forward;
    float32_t tilt_earth_right;
    float32_t tilt_body_forward;
    float32_t tilt_body_right;

    if (dt < 0.0001f) {
        dt = 0.0001f;
    }

    position_error_forward = clamp_float(
        optical_flow_state.hold_position_earth_m[0] - optical_flow_state.position_earth_m[0],
        -OPTFLOW_MAX_POSITION_ERROR_M,
        OPTFLOW_MAX_POSITION_ERROR_M);
    position_error_right = clamp_float(
        optical_flow_state.hold_position_earth_m[1] - optical_flow_state.position_earth_m[1],
        -OPTFLOW_MAX_POSITION_ERROR_M,
        OPTFLOW_MAX_POSITION_ERROR_M);

    Caculate_PID_ALTIDUE(&PID_POSHOLD_X, position_error_forward, 0.0f, dt);
    Caculate_PID_ALTIDUE(&PID_POSHOLD_Y, position_error_right, 0.0f, dt);

    Caculate_PID_Rate_ALTIDUE(&PID_VELHOLD_X, PID_POSHOLD_X.output, optical_flow_state.velocity_earth_mps[0], dt);
    Caculate_PID_Rate_ALTIDUE(&PID_VELHOLD_Y, PID_POSHOLD_Y.output, optical_flow_state.velocity_earth_mps[1], dt);

    yaw_rad = Complimentary_Filter.Euler_Angle_Deg[2] * (float32_t)DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    tilt_earth_forward = PID_VELHOLD_X.output;
    tilt_earth_right = PID_VELHOLD_Y.output;

    tilt_body_forward = cos_yaw * tilt_earth_forward + sin_yaw * tilt_earth_right;
    tilt_body_right = -sin_yaw * tilt_earth_forward + cos_yaw * tilt_earth_right;

    optical_flow_state.correction_deg[0] = clamp_float(
        tilt_body_right,
        -PID_VELHOLD_Y.max_output,
        PID_VELHOLD_Y.max_output);
    optical_flow_state.correction_deg[1] = clamp_float(
        -tilt_body_forward,
        -PID_VELHOLD_X.max_output,
        PID_VELHOLD_X.max_output);
}

static void MIX_THROTTLE(float32_t thr, float32_t *moment, float32_t *m)
{
    int i;

    /*
     * Motor layout: M1(FR CCW), M2(BR CW), M3(BL CCW), M4(FL CW)
     * Yaw right (+): CCW motors speed up, CW motors slow down.
     * Pitch up (+moment[1]): front motors increase, rear motors decrease.
     */
    m[0] = thr - moment[0] - moment[1] - moment[2];
    m[1] = thr - moment[0] + moment[1] + moment[2];
    m[2] = thr + moment[0] + moment[1] - moment[2];
    m[3] = thr + moment[0] - moment[1] + moment[2];

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
        TIM3->CCR1 = PWM_TIMER[1];
        TIM3->CCR2 = PWM_TIMER[0];
        TIM4->CCR1 = PWM_TIMER[3];
        TIM4->CCR2 = PWM_TIMER[2];
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
    FlightController_ResetOpticalFlowHold();
}

void FlightController_ResetOpticalFlowHold(void)
{
    FlightController_OpticalFlowState_t cleared_state = {0};

    optical_flow_state = cleared_state;
    optical_flow_lpf_inited = 0U;
    reset_optical_flow_controller();
}

void FlightController_UpdateOpticalFlowVelocity(float32_t velocity_x_body_mps,
                                                float32_t velocity_y_body_mps,
                                                float32_t altitude_m,
                                                float32_t quality,
                                                float32_t dt)
{
    float32_t yaw_rad;
    float32_t cos_yaw;
    float32_t sin_yaw;
    float32_t velocity_body_x;
    float32_t velocity_body_y;

    optical_flow_state.quality = normalize_optical_flow_quality(quality);
    optical_flow_state.altitude_m = altitude_m;

    if ((dt < 0.001f)
        || (dt > 0.1f)
        || (altitude_m < OPTFLOW_MIN_ALTITUDE_M)
        || (altitude_m > OPTFLOW_MAX_ALTITUDE_M)
        || (optical_flow_state.quality < OPTFLOW_MIN_QUALITY)) {
        invalidate_optical_flow_hold();
        return;
    }

    velocity_body_x = clamp_float(
        velocity_x_body_mps,
        -OPTFLOW_MAX_BODY_VELOCITY_MPS,
        OPTFLOW_MAX_BODY_VELOCITY_MPS);
    velocity_body_y = clamp_float(
        velocity_y_body_mps,
        -OPTFLOW_MAX_BODY_VELOCITY_MPS,
        OPTFLOW_MAX_BODY_VELOCITY_MPS);

    if (optical_flow_lpf_inited == 0U) {
        optical_flow_state.velocity_body_mps[0] = velocity_body_x;
        optical_flow_state.velocity_body_mps[1] = velocity_body_y;
        optical_flow_lpf_inited = 1U;
    } else {
        optical_flow_state.velocity_body_mps[0] += OPTFLOW_VELOCITY_LPF_GAIN
            * (velocity_body_x - optical_flow_state.velocity_body_mps[0]);
        optical_flow_state.velocity_body_mps[1] += OPTFLOW_VELOCITY_LPF_GAIN
            * (velocity_body_y - optical_flow_state.velocity_body_mps[1]);
    }

    yaw_rad = Complimentary_Filter.Euler_Angle_Deg[2] * (float32_t)DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    optical_flow_state.velocity_earth_mps[0] = cos_yaw * optical_flow_state.velocity_body_mps[0]
        - sin_yaw * optical_flow_state.velocity_body_mps[1];
    optical_flow_state.velocity_earth_mps[1] = sin_yaw * optical_flow_state.velocity_body_mps[0]
        + cos_yaw * optical_flow_state.velocity_body_mps[1];

    optical_flow_state.position_earth_m[0] += optical_flow_state.velocity_earth_mps[0] * dt;
    optical_flow_state.position_earth_m[1] += optical_flow_state.velocity_earth_mps[1] * dt;
    optical_flow_state.data_age_s = 0.0f;
    optical_flow_state.healthy = 1U;
}

void FlightController_GetOpticalFlowState(FlightController_OpticalFlowState_t *state_out)
{
    if (state_out != 0) {
        *state_out = optical_flow_state;
    }
}

void MPC(void)
{
    float32_t feedback[3];
    float32_t stick_roll;
    float32_t stick_pitch;
    float32_t stick_yaw;
    float32_t real_dt = MPU6500_DATA.dt;

    if (real_dt > 0.01f) {
        real_dt = 0.01f;
    }
    if (real_dt < 0.001f) {
        real_dt = 0.001f;
    }

    age_optical_flow_data(real_dt);

    if (RC_Raw_Throttle > 2000U) {
        Throttle = 2000.0f;
    } else if (RC_Raw_Throttle < 1000U) {
        Throttle = 1000.0f;
    } else {
        Throttle = (float32_t)RC_Raw_Throttle;
    }

    {
        MPC_Status_t current_mode = (RC_Raw_SW_Mode > 1500U) ? HOVER : RATE_MODE;
        if (current_mode != last_MPC_Status) {
            MPC_Status = current_mode;
            last_MPC_Status = current_mode;
            reset_pid_request = 1U;
            reset_optical_flow_controller();
        }
    }

    if (RC_Raw_SW_Arm > 1500U) {
        if ((ARM_Status == NOT_ARM) && (Throttle < 1150.0f)) {
            ARM_Status = ARM;
            enable_motor = 1U;
            RESET_ALL_PID();
            angle_desired[0] = 0.0f;
            angle_desired[1] = 0.0f;
            angle_desired[2] = Complimentary_Filter.Euler_Angle_Deg[2];
            reset_pid_request = 1U;
        }
    } else {
        ARM_Status = NOT_ARM;
        enable_motor = 0U;
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
        float32_t angle_step;
        uint8_t pilot_demands_translation = (uint8_t)(
            (fabsf(stick_roll) > OPTFLOW_RELEASE_STICK_THRESHOLD_US)
            || (fabsf(stick_pitch) > OPTFLOW_RELEASE_STICK_THRESHOLD_US));
        uint8_t pilot_centered_for_hold = (uint8_t)(
            (fabsf(stick_roll) < OPTFLOW_CAPTURE_STICK_THRESHOLD_US)
            && (fabsf(stick_pitch) < OPTFLOW_CAPTURE_STICK_THRESHOLD_US));

        angle_desired[0] = stick_roll * MANUAL_TILT_SCALE_DEG;
        angle_desired[1] = -stick_pitch * MANUAL_TILT_SCALE_DEG;

        if ((optical_flow_state.healthy != 0U)
            && ((optical_flow_state.hold_locked != 0U) || (pilot_centered_for_hold != 0U))) {
            if (pilot_demands_translation != 0U) {
                reset_optical_flow_controller();
            } else {
                if (optical_flow_state.hold_locked == 0U) {
                    capture_optical_flow_hold_position();
                }
                compute_optical_flow_correction(real_dt);
                angle_desired[0] += optical_flow_state.correction_deg[0];
                angle_desired[1] += optical_flow_state.correction_deg[1];
            }
        } else {
            reset_optical_flow_controller();
        }

        angle_desired[0] = clamp_float(angle_desired[0], -MAX_ANGLE_TARGET_DEG, MAX_ANGLE_TARGET_DEG);
        angle_desired[1] = clamp_float(angle_desired[1], -MAX_ANGLE_TARGET_DEG, MAX_ANGLE_TARGET_DEG);

        angle_step = (stick_yaw / 500.0f) * MANUAL_YAW_SPEED_DPS * real_dt;
        angle_desired[2] += angle_step;

        if (angle_desired[2] > 180.0f) {
            angle_desired[2] -= 360.0f;
        }
        if (angle_desired[2] < -180.0f) {
            angle_desired[2] += 360.0f;
        }
    } else {
        reset_optical_flow_controller();
        angle_rate_desired[0] = stick_roll * MANUAL_RATE_SCALE_DPS;
        angle_rate_desired[1] = -stick_pitch * MANUAL_RATE_SCALE_DPS;
        angle_rate_desired[2] = stick_yaw * MANUAL_RATE_SCALE_DPS;
    }

    if (RC_Raw_Throttle < 950U) {
        if (ARM_Status == ARM) {
            MPC_Status = HOVER;
            angle_desired[0] = 0.0f;
            angle_desired[1] = 0.0f;
            reset_optical_flow_controller();
            Throttle -= 0.2f;
            if (Throttle < 1100.0f) {
                ARM_Status = NOT_ARM;
                enable_motor = 0U;
                Throttle = 1000.0f;
            }
        }
    }

    if (ARM_Status == ARM) {
        if (Throttle < 1150.0f) {
            PID_ROLL.integral *= 0.98f;
            PID_PITCH.integral *= 0.98f;
            PID_YAW.integral *= 0.98f;
            PID_RATE_ROLL.integral *= 0.98f;
            PID_RATE_PITCH.integral *= 0.98f;
            PID_RATE_YAW.integral *= 0.98f;
        }

        switch (MPC_Status) {
        case RATE_MODE:
            if (reset_pid_request != 0U) {
                PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                reset_pid_request = 0U;
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

            if (reset_pid_request != 0U) {
                PID_RATE_ROLL.prev_setpoint = angle_rate_desired[0];
                PID_RATE_PITCH.prev_setpoint = angle_rate_desired[1];
                PID_RATE_YAW.prev_setpoint = angle_rate_desired[2];
                reset_pid_request = 0U;
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
            PWM_MOTOR[i] = 1000.0f;
            PWM_TIMER[i] = 1000U;
        }
        Control_Motor();
        RESET_ALL_PID();

        angle_rate_desired[0] = 0.0f;
        angle_rate_desired[1] = 0.0f;
        angle_rate_desired[2] = 0.0f;
    }
}
