#ifndef CORE_APP_ESTIMATION_COMPLEMENTARYFILTER_FILTER_H_
#define CORE_APP_ESTIMATION_COMPLEMENTARYFILTER_FILTER_H_

#include "main.h"
#include "arm_math.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943
#endif

typedef struct {
    float32_t gyro[3];
    float32_t acc[3];
    uint32_t timestamp;
} IMU_RAW_DATA_t;

typedef struct {
    float32_t mag[3];
    uint32_t timestamp;
} MAG_RAW_DATA_t;

typedef struct {
    float32_t mag_uT[3];
    uint32_t timestamp;
} MAG_DATA_t;

typedef struct {
    float32_t alt;
    uint32_t timestamp;
} BARO_RAW_DATA_t;

typedef struct {
    float32_t w[3];
    float32_t acc[3];
    float32_t dt;
    uint32_t timestamp;
} IMU_Data_t;

typedef enum {
    Fusion_START,
    Fusion_RUN,
    Fusion_STOP,
    Fusion_RESET
} Fusion_Status_t;

typedef struct {
    float32_t alpha[3];
    float32_t Euler_Angle_Deg[3];
    float32_t Euler_Angle_Rad[3];
    float32_t q[4];
    uint32_t update_count;
    uint32_t predict_count;
    Fusion_Status_t status;
    uint8_t Fusion_OK;
    float32_t ypr[3];
    float32_t Offset_Deg[3];
    float32_t Offset_Rad[3];
} Complimentary_Filter_t;

void Complimentary_Filter_Reset(Complimentary_Filter_t *imu);
void Complimentary_Filter_Predict(Complimentary_Filter_t *imu, IMU_Data_t *imu_data);
void Complimentary_Filter_Update(Complimentary_Filter_t *imu, MAG_DATA_t *mag);

#endif /* CORE_APP_ESTIMATION_COMPLEMENTARYFILTER_FILTER_H_ */
