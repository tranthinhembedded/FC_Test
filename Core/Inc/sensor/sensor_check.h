#ifndef CORE_APP_SENSORS_SENSOR_CHECK_H_
#define CORE_APP_SENSORS_SENSOR_CHECK_H_

#include <stdint.h>

#define SENSOR_CHECK_I2C_SENSOR_COUNT       2U
#define SENSOR_CHECK_I2C_SENSOR_MAG_INDEX   0U
#define SENSOR_CHECK_I2C_SENSOR_BARO_INDEX  1U

extern volatile uint32_t i2c_probe_count;
extern volatile uint32_t i2c_last_probe_tick_ms;
extern volatile uint8_t icm20602_bus_is_spi;
extern volatile uint8_t icm20602_bus_addr_7bit;
extern volatile uint8_t icm20602_bus_addr_8bit;
extern volatile uint8_t icm20602_who_am_i_reg_addr;
extern volatile uint8_t icm20602_expected_who_am_i;
extern volatile uint8_t icm20602_connected;
extern volatile uint8_t icm20602_ready;
extern volatile uint8_t icm20602_last_read_ok;
extern volatile uint8_t icm20602_who_am_i;

extern volatile uint8_t i2c1_mag_1e_ready;
extern volatile uint8_t i2c1_mag_0d_ready;
extern volatile uint8_t i2c1_mag_id_read_ok;
extern volatile uint8_t i2c1_mag_id_a;
extern volatile uint8_t i2c1_mag_id_b;
extern volatile uint8_t i2c1_mag_id_c;
extern volatile uint8_t i2c1_mag_hmc_signature_ok;
extern volatile uint8_t i2c1_mag_qmc_id_read_ok;
extern volatile uint8_t i2c1_mag_qmc_chip_id;
extern volatile uint8_t i2c1_mag_qmc_detected;
extern volatile uint8_t mag_sensor_type;

extern volatile uint8_t baro_detected_bus;
extern volatile uint8_t baro_ready;
extern volatile uint8_t baro_id_read_ok;
extern volatile uint8_t baro_chip_id;
extern volatile uint8_t baro_addr_7bit;
extern volatile uint8_t baro_addr_8bit;
extern volatile uint8_t baro_is_bme280;

extern volatile uint8_t i2c1_baro_76_ack;
extern volatile uint8_t i2c1_baro_77_ack;
extern volatile uint8_t i2c1_baro_ready;
extern volatile uint8_t i2c1_baro_id_read_ok;
extern volatile uint8_t i2c1_baro_chip_id;
extern volatile uint8_t i2c1_baro_addr_7bit;
extern volatile uint8_t i2c1_baro_addr_8bit;
extern volatile uint8_t i2c1_baro_is_bme280;

extern volatile uint8_t i2c2_baro_76_ack;
extern volatile uint8_t i2c2_baro_77_ack;
extern volatile uint8_t i2c2_baro_ready;
extern volatile uint8_t i2c2_baro_id_read_ok;
extern volatile uint8_t i2c2_baro_chip_id;
extern volatile uint8_t i2c2_baro_addr_7bit;
extern volatile uint8_t i2c2_baro_addr_8bit;
extern volatile uint8_t i2c2_baro_is_bme280;

extern volatile uint8_t i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_COUNT];
extern volatile uint8_t i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_COUNT];
extern volatile uint8_t i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_COUNT];
extern volatile uint8_t i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_COUNT];
extern volatile uint8_t i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_COUNT];

extern volatile uint8_t mag_detected_bus;
extern volatile uint8_t imu_mag_connected;
extern volatile uint8_t all_sensors_connected;
extern volatile uint8_t runtime_sensors_ok;
extern volatile uint32_t led_blink_period_ms;

void SensorCheck_RunStartupProbe(void);
void SensorCheck_UpdateHeartbeat(uint32_t now_ms);

#endif /* CORE_APP_SENSORS_SENSOR_CHECK_H_ */
