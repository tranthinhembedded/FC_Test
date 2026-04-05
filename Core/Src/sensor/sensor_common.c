#include "sensor/sensor_common.h"

IMU_RAW_DATA_t MPU6500_RAW_DATA;
IMU_Data_t MPU6500_DATA;

MAG_RAW_DATA_t HMC5883L_RAW_DATA;
MAG_DATA_t HMC5883L_DATA;

MagCal_Simple_t MagCal = {
    .S = 1.0f,
    .state = MAG_CAL_DONE,
    .samples_target = 5000,
    .offset = {4.7840004f, -3.45000076f, 7.36000061f},
    .scale = {0.937098265f, 0.954184234f, 1.13012183f}
};

Complimentary_Filter_t Complimentary_Filter = {
    .alpha = {0.99f, 0.99f, 0.96f},
    .Offset_Deg = {0.0f, 0.0f, -28.97f},
    .Offset_Rad = {0.0f, 0.0f, -28.97f * (float32_t)DEG_TO_RAD}
};

PID_ALTIDUE_t PID_RATE_ROLL = {
    .alpha_lpf = 0.88f, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 0.700993f, .ki = 1.265000f, .kd = 0.112501f, .d_limit = 22.5f,
};

PID_ALTIDUE_t PID_RATE_PITCH = {
    .alpha_lpf = 0.88f, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 0.700993f, .ki = 1.265000f, .kd = 0.112501f, .d_limit = 22.5f,
};

PID_ALTIDUE_t PID_RATE_YAW = {
    .alpha_lpf = 0.88f, .feed_forward = 0.05f, .i_limit = 75, .max_output = 400,
    .kp = 1.456010f, .ki = 1.401011f, .kd = 0.138702f, .d_limit = 22.5f,
};

PID_ALTIDUE_t PID_ROLL = {
    .alpha_lpf = 0.88f, .feed_forward = 0, .i_limit = 65, .max_output = 150,
    .kp = 1.7f, .ki = 0.007f, .kd = 1.8f, .d_limit = 22.5f,
};

PID_ALTIDUE_t PID_PITCH = {
    .alpha_lpf = 0.88f, .feed_forward = 0, .i_limit = 65, .max_output = 150,
    .kp = 1.7f, .ki = 0.007f, .kd = 1.8f, .d_limit = 22.5f,
};

PID_ALTIDUE_t PID_YAW = {
    .alpha_lpf = 0.88f, .feed_forward = 0, .i_limit = 60, .max_output = 120,
    .kp = 1.5f, .ki = 0.001f, .kd = 0.0f, .d_limit = 22.5f,
};

MPC_Status_t MPC_Status = HOVER;
ARM_Status_t ARM_Status = NOT_ARM;

int16_t raw_acc[3];
int16_t raw_gyro[3];
float32_t acc_phys[3];
float32_t gyro_phys[3];
uint8_t enable_motor = 0;

float32_t Throttle = 1000.0f;
float32_t Moment[3];
float32_t angle_desired[3] = {0, 0, 0};
float32_t angle_rate_desired[3] = {0, 0, 0};

float32_t PWM_MOTOR[4];
uint32_t PWM_TIMER[4];

float vbat = 11.1f;

float32_t gyro_final[3] = {0.0f, 0.0f, 0.0f};
float32_t acc_filtered[3] = {0.0f, 0.0f, 0.0f};
const float32_t alpha_acc_soft = 0.02f;
const float32_t alpha_gyro = 0.05f;
float32_t mag_filtered[3] = {0.0f, 0.0f, 0.0f};
uint8_t mag_lpf_inited = 0;

float32_t gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float32_t accel_bias[3] = {0.0f, 0.0f, 0.0f};
uint8_t is_calibrated = 0;

volatile uint32_t RC_Raw_Roll = 0;
volatile uint32_t RC_Raw_Pitch = 0;
volatile uint32_t RC_Raw_Throttle = 0;
volatile uint32_t RC_Raw_Yaw = 0;
volatile uint32_t RC_Raw_SW_Arm = 0;
volatile uint32_t RC_Raw_SW_Mode = 0;
