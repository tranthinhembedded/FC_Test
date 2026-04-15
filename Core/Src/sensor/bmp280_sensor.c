#include "sensor/bmp280_sensor.h"

#include "platform/delay.h"
#include "platform/i2c.h"

#define BMP280_I2C_ADDR_76            (0x76U << 1)
#define BMP280_I2C_ADDR_77            (0x77U << 1)
#define BMP280_REG_ID                 0xD0U
#define BMP280_REG_RESET              0xE0U
#define BMP280_REG_CTRL_MEAS          0xF4U
#define BMP280_REG_CONFIG             0xF5U
#define BMP280_REG_PRESS_MSB          0xF7U
#define BMP280_CHIP_ID                0x58U
#define BME280_CHIP_ID                0x60U
#define BMP280_RESET_VALUE            0xB6U

static uint8_t s_bmp280_ready = 0U;
static uint8_t s_bmp280_is_bme280 = 0U;
static uint8_t s_bmp280_chip_id = 0U;
static uint8_t s_bmp280_i2c_addr = 0U;
static uint8_t s_bmp280_last_read_ok = 0U;

static uint8_t BMP280_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c2, s_bmp280_i2c_addr, reg, 1, data, len, 100) == HAL_OK) ? 1U : 0U;
}

static uint8_t BMP280_WriteRegister(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c2, s_bmp280_i2c_addr, reg, 1, &value, 1, 100) == HAL_OK) ? 1U : 0U;
}

void BMP280_Init(void)
{
    uint8_t id = 0U;
    uint8_t candidate_addr[2] = {BMP280_I2C_ADDR_76, BMP280_I2C_ADDR_77};
    uint32_t i;

    s_bmp280_ready = 0U;
    s_bmp280_is_bme280 = 0U;
    s_bmp280_chip_id = 0U;
    s_bmp280_i2c_addr = 0U;
    s_bmp280_last_read_ok = 0U;

    for (i = 0U; i < 2U; i++) {
        s_bmp280_i2c_addr = candidate_addr[i];
        if (HAL_I2C_Mem_Read(&hi2c2, s_bmp280_i2c_addr, BMP280_REG_ID, 1, &id, 1, 100) != HAL_OK) {
            continue;
        }

        s_bmp280_chip_id = id;
        if (id == BMP280_CHIP_ID || id == BME280_CHIP_ID) {
            s_bmp280_ready = 1U;
            s_bmp280_is_bme280 = (id == BME280_CHIP_ID) ? 1U : 0U;
            break;
        }
    }

    if (!s_bmp280_ready) {
        s_bmp280_i2c_addr = 0U;
        return;
    }

    if (!BMP280_WriteRegister(BMP280_REG_RESET, BMP280_RESET_VALUE)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
        return;
    }
    Delay_ms_blocking(10U);

    if (!BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, 0x27U)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
        return;
    }

    if (!BMP280_WriteRegister(BMP280_REG_CONFIG, 0x10U)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
    }
}

uint8_t BMP280_IsReady(void)
{
    return s_bmp280_ready;
}

uint8_t BMP280_IsBme280(void)
{
    return s_bmp280_is_bme280;
}

uint8_t BMP280_GetChipId(void)
{
    return s_bmp280_chip_id;
}

uint8_t BMP280_GetAddress(void)
{
    return s_bmp280_i2c_addr;
}

uint8_t BMP280_GetLastReadOk(void)
{
    return s_bmp280_last_read_ok;
}

uint8_t BMP280_ReadRaw(int32_t *pressure_adc, int32_t *temp_adc)
{
    uint8_t raw[6];

    s_bmp280_last_read_ok = 0U;

    if (!s_bmp280_ready) {
        return 0U;
    }

    if (!BMP280_ReadRegisters(BMP280_REG_PRESS_MSB, raw, sizeof(raw))) {
        return 0U;
    }

    if (pressure_adc != 0) {
        *pressure_adc = (int32_t)((((uint32_t)raw[0]) << 12)
            | (((uint32_t)raw[1]) << 4)
            | (((uint32_t)raw[2]) >> 4));
    }

    if (temp_adc != 0) {
        *temp_adc = (int32_t)((((uint32_t)raw[3]) << 12)
            | (((uint32_t)raw[4]) << 4)
            | (((uint32_t)raw[5]) >> 4));
    }

    s_bmp280_last_read_ok = 1U;
    return 1U;
}
