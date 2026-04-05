#ifndef CORE_APP_STATE_APPSTATE_APP_STATE_H_
#define CORE_APP_STATE_APPSTATE_APP_STATE_H_

#include "control/pid.h"
#include "sensor/complementary_filter.h"
#include "sensor/mag_calibration.h"

typedef enum {
    HOVER,
    RATE_MODE
} MPC_Status_t;

typedef enum {
    ARM,
    NOT_ARM
} ARM_Status_t;

extern IMU_RAW_DATA_t MPU6500_RAW_DATA;
extern IMU_Data_t MPU6500_DATA;

extern MAG_RAW_DATA_t HMC5883L_RAW_DATA;
extern MAG_DATA_t HMC5883L_DATA;

extern MagCal_Simple_t MagCal;
extern Complimentary_Filter_t Complimentary_Filter;

extern PID_ALTIDUE_t PID_RATE_ROLL;
extern PID_ALTIDUE_t PID_RATE_PITCH;
extern PID_ALTIDUE_t PID_RATE_YAW;
extern PID_ALTIDUE_t PID_ROLL;
extern PID_ALTIDUE_t PID_PITCH;
extern PID_ALTIDUE_t PID_YAW;

extern MPC_Status_t MPC_Status;
extern ARM_Status_t ARM_Status;

extern int16_t raw_acc[3];
extern int16_t raw_gyro[3];
extern float32_t acc_phys[3];
extern float32_t gyro_phys[3];
extern uint8_t enable_motor;

extern float32_t Throttle;
extern float32_t Moment[3];
extern float32_t angle_desired[3];
extern float32_t angle_rate_desired[3];

extern float32_t PWM_MOTOR[4];
extern uint32_t PWM_TIMER[4];

extern float vbat;

extern float32_t gyro_final[3];
extern float32_t acc_filtered[3];
extern const float32_t alpha_acc_soft;
extern const float32_t alpha_gyro;
extern float32_t mag_filtered[3];
extern uint8_t mag_lpf_inited;

extern float32_t gyro_bias[3];
extern float32_t accel_bias[3];
extern uint8_t is_calibrated;

extern volatile uint32_t RC_Raw_Roll;
extern volatile uint32_t RC_Raw_Pitch;
extern volatile uint32_t RC_Raw_Throttle;
extern volatile uint32_t RC_Raw_Yaw;
extern volatile uint32_t RC_Raw_SW_Arm;
extern volatile uint32_t RC_Raw_SW_Mode;

#endif /* CORE_APP_STATE_APPSTATE_APP_STATE_H_ */
