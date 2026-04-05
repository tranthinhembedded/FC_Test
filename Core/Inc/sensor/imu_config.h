#ifndef CORE_APP_SENSORS_IMU_IMU_SENSOR_H_
#define CORE_APP_SENSORS_IMU_IMU_SENSOR_H_

#include <stdint.h>

void ICM20602_Init(void);
void ICM20602_Calibrate(void);
uint8_t ICM20602_IsReady(void);
void IMU_PROCESS(void);

#endif /* CORE_APP_SENSORS_IMU_IMU_SENSOR_H_ */
