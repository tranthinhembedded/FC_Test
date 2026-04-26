#ifndef CORE_APP_SENSORS_BMP280_SENSOR_H_
#define CORE_APP_SENSORS_BMP280_SENSOR_H_

#include <stdint.h>

void BMP280_Init(void);
void BMP280_Service(uint32_t now_ms);
uint8_t BMP280_IsReady(void);
uint8_t BMP280_IsBme280(void);
uint8_t BMP280_GetChipId(void);
uint8_t BMP280_GetAddress(void);
uint8_t BMP280_GetLastReadOk(void);
uint8_t BMP280_ReadRaw(int32_t *pressure_adc, int32_t *temp_adc);
uint8_t BMP280_ReadCompensated(float *pressure_pa, float *temp_c);

#endif /* CORE_APP_SENSORS_BMP280_SENSOR_H_ */
