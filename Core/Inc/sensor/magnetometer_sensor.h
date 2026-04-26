#ifndef CORE_APP_SENSORS_MAGNETOMETER_MAGNETOMETER_SENSOR_H_
#define CORE_APP_SENSORS_MAGNETOMETER_MAGNETOMETER_SENSOR_H_

#include <stdint.h>

#define MAGNETOMETER_TYPE_NONE      0U
#define MAGNETOMETER_TYPE_HMC5883L  1U
#define MAGNETOMETER_TYPE_QMC5883L  2U

void Magnetometer_Init(void);
void Magnetometer_Service(uint32_t now_ms);
uint8_t Magnetometer_IsReady(void);
void Magnetometer_GetId(uint8_t id_out[3]);
uint8_t Magnetometer_GetLastReadOk(void);
uint8_t Magnetometer_GetType(void);
uint8_t Magnetometer_GetAddress7Bit(void);
uint8_t Magnetometer_GetChipId(void);
void COMPASS_PROCESS(void);

#endif /* CORE_APP_SENSORS_MAGNETOMETER_MAGNETOMETER_SENSOR_H_ */
