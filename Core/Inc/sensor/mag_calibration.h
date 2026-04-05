#ifndef CORE_APP_SENSORS_CALIBRATION_CAL_H_
#define CORE_APP_SENSORS_CALIBRATION_CAL_H_

#include "main.h"
#include "arm_math.h"
#include "sensor/complementary_filter.h"

typedef enum {
    MAG_CAL_IDLE = 0,
    MAG_CAL_START,
    MAG_CAL_COLLECTING,
    MAG_CAL_COMPUTE,
    MAG_CAL_DONE
} MagCal_State_t;

typedef struct {
    MagCal_State_t state;
    float32_t S;
    float32_t min[3];
    float32_t max[3];
    float32_t offset[3];
    float32_t scale[3];
    uint32_t samples;
    uint32_t samples_target;
} MagCal_Simple_t;

void Mag_ApplyCalibration(MagCal_Simple_t *c, MAG_RAW_DATA_t *raw, MAG_DATA_t *out);
void MagCal_Update(MagCal_Simple_t *c, MAG_RAW_DATA_t *raw, MAG_DATA_t *out);

#endif /* CORE_APP_SENSORS_CALIBRATION_CAL_H_ */
