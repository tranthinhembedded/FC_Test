#include "sensor/sensor_check.h"

#include "main.h"
#include "i2c.h"
#include "sensor/imu_config.h"

#define SENSOR_CHECK_LED_BLINK_OK_MS  100U
#define SENSOR_CHECK_MAG_ADDR_8BIT     (0x1EU << 1)
#define SENSOR_CHECK_MAG_REG_ID_A      0x0AU

volatile uint32_t i2c_probe_count = 0U;
volatile uint32_t i2c_last_probe_tick_ms = 0U;
volatile uint8_t icm20602_connected = 0U;
volatile uint8_t icm20602_ready = 0U;
volatile uint8_t icm20602_last_read_ok = 0U;
volatile uint8_t icm20602_who_am_i = 0U;

volatile uint8_t i2c1_mag_1e_ready = 0U;
volatile uint8_t i2c1_mag_id_read_ok = 0U;
volatile uint8_t i2c1_mag_id_a = 0U;
volatile uint8_t i2c1_mag_id_b = 0U;
volatile uint8_t i2c1_mag_id_c = 0U;
volatile uint8_t i2c1_mag_hmc_signature_ok = 0U;

volatile uint8_t mag_detected_bus = 0U;
volatile uint8_t imu_mag_connected = 0U;
volatile uint8_t all_sensors_connected = 0U;
volatile uint8_t runtime_sensors_ok = 0U;
volatile uint32_t led_blink_period_ms = 0U;

static uint8_t SensorCheck_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t address)
{
    return (HAL_I2C_IsDeviceReady(hi2c, address, 2U, 20U) == HAL_OK) ? 1U : 0U;
}

static uint8_t SensorCheck_ReadRegister(I2C_HandleTypeDef *hi2c, uint16_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
    return (HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, data, len, 50U) == HAL_OK) ? 1U : 0U;
}

static void SensorCheck_ProbeImu(void)
{
    if (ICM20602_IsReady() == 0U) {
        ICM20602_Init();
    }

    icm20602_ready = ICM20602_IsReady();
    icm20602_who_am_i = ICM20602_GetWhoAmI();

    if (icm20602_ready != 0U) {
        IMU_PROCESS();
    }

    icm20602_last_read_ok = ICM20602_GetLastReadOk();
    icm20602_connected = (uint8_t)((icm20602_ready != 0U) && (icm20602_last_read_ok != 0U));
}

static void SensorCheck_ProbeMag(void)
{
    uint8_t id[3] = {0U, 0U, 0U};

    i2c1_mag_1e_ready = SensorCheck_IsDeviceReady(&hi2c1, SENSOR_CHECK_MAG_ADDR_8BIT);
    i2c1_mag_id_read_ok = 0U;
    i2c1_mag_id_a = 0U;
    i2c1_mag_id_b = 0U;
    i2c1_mag_id_c = 0U;
    i2c1_mag_hmc_signature_ok = 0U;

    if (i2c1_mag_1e_ready == 0U) {
        return;
    }

    if (SensorCheck_ReadRegister(&hi2c1, SENSOR_CHECK_MAG_ADDR_8BIT, SENSOR_CHECK_MAG_REG_ID_A, id, sizeof(id)) == 0U) {
        return;
    }

    i2c1_mag_id_a = id[0];
    i2c1_mag_id_b = id[1];
    i2c1_mag_id_c = id[2];
    i2c1_mag_id_read_ok = 1U;
    if ((id[0] == 'H') && (id[1] == '4') && (id[2] == '3')) {
        i2c1_mag_hmc_signature_ok = 1U;
    }
}

void SensorCheck_RunStartupProbe(void)
{
    i2c_probe_count++;
    i2c_last_probe_tick_ms = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    SensorCheck_ProbeImu();
    SensorCheck_ProbeMag();

    mag_detected_bus = (i2c1_mag_1e_ready != 0U) ? 1U : 0U;
    imu_mag_connected = (uint8_t)((icm20602_connected != 0U)
                               && (i2c1_mag_hmc_signature_ok != 0U));
    all_sensors_connected = imu_mag_connected;
    runtime_sensors_ok = all_sensors_connected;
    led_blink_period_ms = (all_sensors_connected != 0U) ? SENSOR_CHECK_LED_BLINK_OK_MS : 0U;
}

void SensorCheck_UpdateHeartbeat(uint32_t now_ms)
{
    static uint32_t s_last_led_toggle_ms = 0U;

    if (led_blink_period_ms == 0U) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        return;
    }

    if ((uint32_t)(now_ms - s_last_led_toggle_ms) >= led_blink_period_ms) {
        s_last_led_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
