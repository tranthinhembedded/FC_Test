#ifndef CORE_APP_SENSORS_BMP280_SENSOR_H_
#define CORE_APP_SENSORS_BMP280_SENSOR_H_

#include <stdint.h>

void BMP280_Init(void);
uint8_t BMP280_IsReady(void);
uint8_t BMP280_IsBme280(void);
uint8_t BMP280_GetChipId(void);
uint8_t BMP280_GetAddress(void);
uint8_t BMP280_GetLastReadOk(void);
uint8_t BMP280_ReadRaw(int32_t *pressure_adc, int32_t *temp_adc);

#endif /* CORE_APP_SENSORS_BMP280_SENSOR_H_ */
