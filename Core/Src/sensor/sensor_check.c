#include "sensor/sensor_check.h"

#include "main.h"
#include "platform/i2c.h"
#include "sensor/imu_config.h"

#define SENSOR_CHECK_LED_BLINK_OK_MS  100U
#define SENSOR_CHECK_MAG_HMC_ADDR_7BIT 0x1EU
#define SENSOR_CHECK_MAG_HMC_ADDR_8BIT (SENSOR_CHECK_MAG_HMC_ADDR_7BIT << 1)
#define SENSOR_CHECK_MAG_HMC_REG_ID_A  0x0AU
#define SENSOR_CHECK_MAG_QMC_ADDR_7BIT 0x0DU
#define SENSOR_CHECK_MAG_QMC_ADDR_8BIT (SENSOR_CHECK_MAG_QMC_ADDR_7BIT << 1)
#define SENSOR_CHECK_MAG_QMC_REG_ID    0x0DU
#define SENSOR_CHECK_MAG_QMC_CHIP_ID   0xFFU
#define SENSOR_CHECK_MAG_TYPE_NONE     0U
#define SENSOR_CHECK_MAG_TYPE_HMC5883L 1U
#define SENSOR_CHECK_MAG_TYPE_QMC5883L 2U
#define SENSOR_CHECK_BARO_ADDR_76_7BIT 0x76U
#define SENSOR_CHECK_BARO_ADDR_77_7BIT 0x77U
#define SENSOR_CHECK_BARO_ADDR_76_8BIT (SENSOR_CHECK_BARO_ADDR_76_7BIT << 1)
#define SENSOR_CHECK_BARO_ADDR_77_8BIT (SENSOR_CHECK_BARO_ADDR_77_7BIT << 1)
#define SENSOR_CHECK_BARO_REG_ID       0xD0U
#define SENSOR_CHECK_BARO_BMP280_ID    0x58U
#define SENSOR_CHECK_BARO_BME280_ID    0x60U
#define SENSOR_CHECK_ICM20602_WHO_AM_I_REG 0x75U
#define SENSOR_CHECK_ICM20602_WHO_AM_I_ID  0x12U
#define SENSOR_CHECK_I2C_BUS_NONE      0U
#define SENSOR_CHECK_I2C_BUS_1         1U
#define SENSOR_CHECK_I2C_BUS_2         2U

volatile uint32_t i2c_probe_count = 0U;
volatile uint32_t i2c_last_probe_tick_ms = 0U;
volatile uint8_t icm20602_bus_is_spi = 1U;
volatile uint8_t icm20602_bus_addr_7bit = 0U;
volatile uint8_t icm20602_bus_addr_8bit = 0U;
volatile uint8_t icm20602_who_am_i_reg_addr = SENSOR_CHECK_ICM20602_WHO_AM_I_REG;
volatile uint8_t icm20602_expected_who_am_i = SENSOR_CHECK_ICM20602_WHO_AM_I_ID;
volatile uint8_t icm20602_connected = 0U;
volatile uint8_t icm20602_ready = 0U;
volatile uint8_t icm20602_last_read_ok = 0U;
volatile uint8_t icm20602_who_am_i = 0U;

volatile uint8_t i2c1_mag_1e_ready = 0U;
volatile uint8_t i2c1_mag_0d_ready = 0U;
volatile uint8_t i2c1_mag_id_read_ok = 0U;
volatile uint8_t i2c1_mag_id_a = 0U;
volatile uint8_t i2c1_mag_id_b = 0U;
volatile uint8_t i2c1_mag_id_c = 0U;
volatile uint8_t i2c1_mag_hmc_signature_ok = 0U;
volatile uint8_t i2c1_mag_qmc_id_read_ok = 0U;
volatile uint8_t i2c1_mag_qmc_chip_id = 0U;
volatile uint8_t i2c1_mag_qmc_detected = 0U;
volatile uint8_t mag_sensor_type = SENSOR_CHECK_MAG_TYPE_NONE;

volatile uint8_t baro_detected_bus = SENSOR_CHECK_I2C_BUS_NONE;
volatile uint8_t baro_ready = 0U;
volatile uint8_t baro_id_read_ok = 0U;
volatile uint8_t baro_chip_id = 0U;
volatile uint8_t baro_addr_7bit = 0U;
volatile uint8_t baro_addr_8bit = 0U;
volatile uint8_t baro_is_bme280 = 0U;

volatile uint8_t i2c1_baro_76_ack = 0U;
volatile uint8_t i2c1_baro_77_ack = 0U;
volatile uint8_t i2c1_baro_ready = 0U;
volatile uint8_t i2c1_baro_id_read_ok = 0U;
volatile uint8_t i2c1_baro_chip_id = 0U;
volatile uint8_t i2c1_baro_addr_7bit = 0U;
volatile uint8_t i2c1_baro_addr_8bit = 0U;
volatile uint8_t i2c1_baro_is_bme280 = 0U;

volatile uint8_t i2c2_baro_76_ack = 0U;
volatile uint8_t i2c2_baro_77_ack = 0U;
volatile uint8_t i2c2_baro_ready = 0U;
volatile uint8_t i2c2_baro_id_read_ok = 0U;
volatile uint8_t i2c2_baro_chip_id = 0U;
volatile uint8_t i2c2_baro_addr_7bit = 0U;
volatile uint8_t i2c2_baro_addr_8bit = 0U;
volatile uint8_t i2c2_baro_is_bme280 = 0U;

/*
 * Live Expressions helpers:
 *   [0] magnetometer on I2C1 (HMC5883L or QMC5883L)
 *   [1] barometer on detected I2C bus
 */
volatile uint8_t i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_COUNT] = {
    0U,
    0U
};
volatile uint8_t i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_COUNT] = {
    0U,
    0U
};
volatile uint8_t i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_COUNT] = {0U, 0U};
volatile uint8_t i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_COUNT] = {0U, 0U};
volatile uint8_t i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_COUNT] = {0U, 0U};

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

static void SensorCheck_SelectBaroResult(
    uint8_t bus,
    uint8_t ready,
    uint8_t id_read_ok,
    uint8_t chip_id,
    uint8_t addr_7bit,
    uint8_t addr_8bit,
    uint8_t is_bme280)
{
    baro_detected_bus = bus;
    baro_ready = ready;
    baro_id_read_ok = id_read_ok;
    baro_chip_id = chip_id;
    baro_addr_7bit = addr_7bit;
    baro_addr_8bit = addr_8bit;
    baro_is_bme280 = is_bme280;
}

static void SensorCheck_ProbeBaroBus(
    I2C_HandleTypeDef *hi2c,
    volatile uint8_t *addr_76_ack,
    volatile uint8_t *addr_77_ack,
    volatile uint8_t *baro_ready_out,
    volatile uint8_t *baro_id_read_ok_out,
    volatile uint8_t *baro_chip_id_out,
    volatile uint8_t *baro_addr_7bit_out,
    volatile uint8_t *baro_addr_8bit_out,
    volatile uint8_t *baro_is_bme280_out)
{
    const uint8_t candidate_addr_7bit[2] = {
        SENSOR_CHECK_BARO_ADDR_76_7BIT,
        SENSOR_CHECK_BARO_ADDR_77_7BIT
    };
    const uint8_t candidate_addr_8bit[2] = {
        SENSOR_CHECK_BARO_ADDR_76_8BIT,
        SENSOR_CHECK_BARO_ADDR_77_8BIT
    };
    volatile uint8_t *ack_out[2] = {addr_76_ack, addr_77_ack};
    uint8_t id = 0U;
    uint32_t i;

    *addr_76_ack = 0U;
    *addr_77_ack = 0U;
    *baro_ready_out = 0U;
    *baro_id_read_ok_out = 0U;
    *baro_chip_id_out = 0U;
    *baro_addr_7bit_out = 0U;
    *baro_addr_8bit_out = 0U;
    *baro_is_bme280_out = 0U;

    for (i = 0U; i < 2U; i++) {
        *ack_out[i] = SensorCheck_IsDeviceReady(hi2c, candidate_addr_8bit[i]);
        if (*ack_out[i] == 0U) {
            continue;
        }

        if (SensorCheck_ReadRegister(hi2c, candidate_addr_8bit[i], SENSOR_CHECK_BARO_REG_ID, &id, 1U) == 0U) {
            continue;
        }

        *baro_id_read_ok_out = 1U;
        *baro_chip_id_out = id;
        *baro_addr_7bit_out = candidate_addr_7bit[i];
        *baro_addr_8bit_out = candidate_addr_8bit[i];

        if ((id == SENSOR_CHECK_BARO_BMP280_ID) || (id == SENSOR_CHECK_BARO_BME280_ID)) {
            *baro_ready_out = 1U;
            *baro_is_bme280_out = (id == SENSOR_CHECK_BARO_BME280_ID) ? 1U : 0U;
            break;
        }
    }
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
    uint8_t qmc_id = 0U;

    i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_I2C_BUS_NONE;
    i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 0U;
    i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 0U;
    i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 0U;
    i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 0U;

    mag_sensor_type = SENSOR_CHECK_MAG_TYPE_NONE;
    i2c1_mag_1e_ready = SensorCheck_IsDeviceReady(&hi2c1, SENSOR_CHECK_MAG_HMC_ADDR_8BIT);
    i2c1_mag_0d_ready = SensorCheck_IsDeviceReady(&hi2c1, SENSOR_CHECK_MAG_QMC_ADDR_8BIT);
    i2c1_mag_id_read_ok = 0U;
    i2c1_mag_id_a = 0U;
    i2c1_mag_id_b = 0U;
    i2c1_mag_id_c = 0U;
    i2c1_mag_hmc_signature_ok = 0U;
    i2c1_mag_qmc_id_read_ok = 0U;
    i2c1_mag_qmc_chip_id = 0U;
    i2c1_mag_qmc_detected = 0U;

    if (i2c1_mag_1e_ready != 0U) {
        if (SensorCheck_ReadRegister(&hi2c1, SENSOR_CHECK_MAG_HMC_ADDR_8BIT, SENSOR_CHECK_MAG_HMC_REG_ID_A, id, sizeof(id)) != 0U) {
            i2c1_mag_id_a = id[0];
            i2c1_mag_id_b = id[1];
            i2c1_mag_id_c = id[2];
            i2c1_mag_id_read_ok = 1U;
            if ((id[0] == 'H') && (id[1] == '4') && (id[2] == '3')) {
                i2c1_mag_hmc_signature_ok = 1U;
                mag_sensor_type = SENSOR_CHECK_MAG_TYPE_HMC5883L;
                i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_I2C_BUS_1;
                i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_MAG_HMC_ADDR_7BIT;
                i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_MAG_HMC_ADDR_8BIT;
                i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 1U;
                i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = i2c1_mag_id_a;
                return;
            }
        }
    }

    if (i2c1_mag_0d_ready != 0U) {
        if (SensorCheck_ReadRegister(&hi2c1, SENSOR_CHECK_MAG_QMC_ADDR_8BIT, SENSOR_CHECK_MAG_QMC_REG_ID, &qmc_id, 1U) != 0U) {
            i2c1_mag_qmc_id_read_ok = 1U;
            i2c1_mag_qmc_chip_id = qmc_id;
            i2c1_mag_qmc_detected = 1U;
            mag_sensor_type = SENSOR_CHECK_MAG_TYPE_QMC5883L;
            i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_I2C_BUS_1;
            i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_MAG_QMC_ADDR_7BIT;
            i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = SENSOR_CHECK_MAG_QMC_ADDR_8BIT;
            i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = 1U;
            i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] = i2c1_mag_qmc_chip_id;
        }
    }
}

static void SensorCheck_ProbeBaro(void)
{
    SensorCheck_SelectBaroResult(SENSOR_CHECK_I2C_BUS_NONE, 0U, 0U, 0U, 0U, 0U, 0U);
    i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = SENSOR_CHECK_I2C_BUS_NONE;
    i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = 0U;
    i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = 0U;
    i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = 0U;
    i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = 0U;

    SensorCheck_ProbeBaroBus(
        &hi2c1,
        &i2c1_baro_76_ack,
        &i2c1_baro_77_ack,
        &i2c1_baro_ready,
        &i2c1_baro_id_read_ok,
        &i2c1_baro_chip_id,
        &i2c1_baro_addr_7bit,
        &i2c1_baro_addr_8bit,
        &i2c1_baro_is_bme280);

    SensorCheck_ProbeBaroBus(
        &hi2c2,
        &i2c2_baro_76_ack,
        &i2c2_baro_77_ack,
        &i2c2_baro_ready,
        &i2c2_baro_id_read_ok,
        &i2c2_baro_chip_id,
        &i2c2_baro_addr_7bit,
        &i2c2_baro_addr_8bit,
        &i2c2_baro_is_bme280);

    if (i2c2_baro_ready != 0U) {
        SensorCheck_SelectBaroResult(
            SENSOR_CHECK_I2C_BUS_2,
            i2c2_baro_ready,
            i2c2_baro_id_read_ok,
            i2c2_baro_chip_id,
            i2c2_baro_addr_7bit,
            i2c2_baro_addr_8bit,
            i2c2_baro_is_bme280);
    } else if (i2c1_baro_ready != 0U) {
        SensorCheck_SelectBaroResult(
            SENSOR_CHECK_I2C_BUS_1,
            i2c1_baro_ready,
            i2c1_baro_id_read_ok,
            i2c1_baro_chip_id,
            i2c1_baro_addr_7bit,
            i2c1_baro_addr_8bit,
            i2c1_baro_is_bme280);
    } else if (i2c2_baro_id_read_ok != 0U) {
        SensorCheck_SelectBaroResult(
            SENSOR_CHECK_I2C_BUS_2,
            0U,
            i2c2_baro_id_read_ok,
            i2c2_baro_chip_id,
            i2c2_baro_addr_7bit,
            i2c2_baro_addr_8bit,
            0U);
    } else if (i2c1_baro_id_read_ok != 0U) {
        SensorCheck_SelectBaroResult(
            SENSOR_CHECK_I2C_BUS_1,
            0U,
            i2c1_baro_id_read_ok,
            i2c1_baro_chip_id,
            i2c1_baro_addr_7bit,
            i2c1_baro_addr_8bit,
            0U);
    }

    i2c_sensor_bus[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = baro_detected_bus;
    i2c_sensor_addr_7bit[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = baro_addr_7bit;
    i2c_sensor_addr_8bit[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = baro_addr_8bit;
    i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = baro_ready;
    i2c_sensor_chip_id[SENSOR_CHECK_I2C_SENSOR_BARO_INDEX] = baro_chip_id;
}

void SensorCheck_RunStartupProbe(void)
{
    i2c_probe_count++;
    i2c_last_probe_tick_ms = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    SensorCheck_ProbeImu();
    SensorCheck_ProbeMag();
    SensorCheck_ProbeBaro();

    mag_detected_bus = (i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] != 0U) ? 1U : 0U;
    imu_mag_connected = (uint8_t)((icm20602_connected != 0U)
                               && (i2c_sensor_ready[SENSOR_CHECK_I2C_SENSOR_MAG_INDEX] != 0U));
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
