#ifndef CORE_APP_SENSORS_MAHONY_MAHONY_H_
#define CORE_APP_SENSORS_MAHONY_MAHONY_H_

#include <stdint.h>

#include "sensor/complementary_filter.h"

typedef struct {
    float twoKp;
    float twoKi;
    float q0;
    float q1;
    float q2;
    float q3;
    float integralFBx;
    float integralFBy;
    float integralFBz;
    float roll_trim_rad;
    float pitch_trim_rad;
    uint32_t predict_count;
    uint32_t update_count;
    uint8_t initialized;
} MahonyAHRS_t;

void MahonyAHRS_Reset(MahonyAHRS_t *ahrs);
void MahonyAHRS_SetGains(MahonyAHRS_t *ahrs, float kp, float ki);
void MahonyAHRS_Update(MahonyAHRS_t *ahrs,
    const IMU_Data_t *imu,
    const MAG_DATA_t *mag,
    uint8_t mag_valid,
    Complimentary_Filter_t *output);

#endif /* CORE_APP_SENSORS_MAHONY_MAHONY_H_ */
