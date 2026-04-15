#ifndef CORE_APP_SENSORS_SENSOR_CHECK_H_
#define CORE_APP_SENSORS_SENSOR_CHECK_H_

#include <stdint.h>

extern volatile uint32_t i2c_probe_count;
extern volatile uint32_t i2c_last_probe_tick_ms;
extern volatile uint8_t icm20602_connected;
extern volatile uint8_t icm20602_ready;
extern volatile uint8_t icm20602_last_read_ok;
extern volatile uint8_t icm20602_who_am_i;

extern volatile uint8_t i2c1_mag_1e_ready;
extern volatile uint8_t i2c1_mag_id_read_ok;
extern volatile uint8_t i2c1_mag_id_a;
extern volatile uint8_t i2c1_mag_id_b;
extern volatile uint8_t i2c1_mag_id_c;
extern volatile uint8_t i2c1_mag_hmc_signature_ok;

extern volatile uint8_t mag_detected_bus;
extern volatile uint8_t imu_mag_connected;
extern volatile uint8_t all_sensors_connected;
extern volatile uint8_t runtime_sensors_ok;
extern volatile uint32_t led_blink_period_ms;

void SensorCheck_RunStartupProbe(void);
void SensorCheck_UpdateHeartbeat(uint32_t now_ms);

#endif /* CORE_APP_SENSORS_SENSOR_CHECK_H_ */
