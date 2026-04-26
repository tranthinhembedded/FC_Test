#include "sensor/bmp280_sensor.h"

#include "platform/delay.h"
#include "platform/i2c.h"
#include "sensor/sensor_common.h"

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
#define BMP280_RECOVERY_FAIL_THRESHOLD 3U
#define BMP280_RECOVERY_COOLDOWN_MS   250U
#define BMP280_SERVICE_INTERVAL_MS    500U
#define BMP280_I2C_INIT_TIMEOUT_MS    5U
#define BMP280_I2C_RUNTIME_TIMEOUT_MS 1U

static uint8_t s_bmp280_ready = 0U;
static uint8_t s_bmp280_is_bme280 = 0U;
static uint8_t s_bmp280_chip_id = 0U;
static uint8_t s_bmp280_i2c_addr = 0U;
static uint8_t s_bmp280_last_read_ok = 0U;
static uint8_t s_bmp280_fail_streak = 0U;
static uint8_t s_bmp280_recovery_pending = 0U;
static uint32_t s_bmp280_last_recovery_ms = 0U;
static uint32_t s_bmp280_last_service_ms = 0U;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

static BMP280_CalibData s_calib_data;
static I2C_HandleTypeDef *s_bmp280_hi2c = 0;

static void BMP280_ResetState(void)
{
    s_bmp280_ready = 0U;
    s_bmp280_is_bme280 = 0U;
    s_bmp280_chip_id = 0U;
    s_bmp280_i2c_addr = 0U;
    s_bmp280_last_read_ok = 0U;
    s_bmp280_fail_streak = 0U;
    s_bmp280_recovery_pending = 0U;
    s_bmp280_hi2c = 0;
}

static uint8_t BMP280_ReadRegistersWithTimeout(uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (!s_bmp280_hi2c) return 0U;
    return I2C_Mem_ReadRecover(s_bmp280_hi2c, s_bmp280_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);
}

static uint8_t BMP280_WriteRegisterWithTimeout(uint8_t reg, uint8_t value, uint32_t timeout_ms)
{
    if (!s_bmp280_hi2c) return 0U;
    return I2C_Mem_WriteRecover(s_bmp280_hi2c, s_bmp280_i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, timeout_ms);
}

static uint8_t BMP280_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
    return BMP280_ReadRegistersWithTimeout(reg, data, len, BMP280_I2C_RUNTIME_TIMEOUT_MS);
}

static void BMP280_AttemptRecovery(void)
{
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;

    s_bmp280_last_recovery_ms = HAL_GetTick();

    if (s_bmp280_hi2c != 0) {
        (void)I2C_BusRecover(s_bmp280_hi2c);
    } else {
        (void)I2C_BusRecover(&hi2c1);
        (void)I2C_BusRecover(&hi2c2);
    }

    Delay_ms_blocking(2U);
    BMP280_Init();
}

static void BMP280_HandleReadFailure(void)
{
    s_bmp280_last_read_ok = 0U;

    if (s_bmp280_fail_streak < 0xFFU) {
        s_bmp280_fail_streak++;
    }

    if ((s_bmp280_fail_streak >= BMP280_RECOVERY_FAIL_THRESHOLD)
        && ((uint32_t)(HAL_GetTick() - s_bmp280_last_recovery_ms) >= BMP280_RECOVERY_COOLDOWN_MS)) {
        s_bmp280_fail_streak = 0U;
        s_bmp280_ready = 0U;
        s_bmp280_recovery_pending = 1U;
    }
}

static void BMP280_HandleReadSuccess(void)
{
    s_bmp280_last_read_ok = 1U;
    s_bmp280_fail_streak = 0U;
}

void BMP280_Init(void)
{
    uint8_t id = 0U;
    uint8_t candidate_addr[2] = {BMP280_I2C_ADDR_76, BMP280_I2C_ADDR_77};
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;
    I2C_HandleTypeDef *candidate_bus[2] = {&hi2c1, &hi2c2};
    uint32_t i, b;

    BMP280_ResetState();

    for (b = 0U; b < 2U; b++) {
        for (i = 0U; i < 2U; i++) {
            s_bmp280_i2c_addr = candidate_addr[i];
            s_bmp280_hi2c = candidate_bus[b];
            if (!I2C_Mem_ReadRecover(s_bmp280_hi2c, s_bmp280_i2c_addr, BMP280_REG_ID, I2C_MEMADD_SIZE_8BIT, &id, 1U, BMP280_I2C_INIT_TIMEOUT_MS)) {
                continue;
            }

            s_bmp280_chip_id = id;
            if (id == BMP280_CHIP_ID || id == BME280_CHIP_ID) {
                s_bmp280_ready = 1U;
                s_bmp280_is_bme280 = (id == BME280_CHIP_ID) ? 1U : 0U;
                goto found_sensor;
            }
        }
    }

found_sensor:

    if (!s_bmp280_ready) {
        s_bmp280_i2c_addr = 0U;
        s_bmp280_hi2c = 0;
        return;
    }

    if (!BMP280_WriteRegisterWithTimeout(BMP280_REG_RESET, BMP280_RESET_VALUE, BMP280_I2C_INIT_TIMEOUT_MS)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
        return;
    }
    Delay_ms_blocking(10U);

    if (!BMP280_WriteRegisterWithTimeout(BMP280_REG_CTRL_MEAS, 0x57U, BMP280_I2C_INIT_TIMEOUT_MS)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
        return;
    }

    if (!BMP280_WriteRegisterWithTimeout(BMP280_REG_CONFIG, 0x10U, BMP280_I2C_INIT_TIMEOUT_MS)) {
        s_bmp280_ready = 0U;
        s_bmp280_i2c_addr = 0U;
        return;
    }

    uint8_t calib[24];
    if (!BMP280_ReadRegistersWithTimeout(0x88U, calib, 24U, BMP280_I2C_INIT_TIMEOUT_MS)) {
        s_bmp280_ready = 0U;
        return;
    }
    s_calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    s_calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    s_calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    s_calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    s_calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    s_calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    s_calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    s_calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    s_calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    s_calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    s_calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    s_calib_data.dig_P9 = (calib[23] << 8) | calib[22];
}

void BMP280_Service(uint32_t now_ms)
{
    if ((s_bmp280_ready != 0U) && (s_bmp280_recovery_pending == 0U)) {
        return;
    }

    if ((enable_motor != 0U) || (ARM_Status == ARM)) {
        return;
    }

    if ((uint32_t)(now_ms - s_bmp280_last_recovery_ms) < BMP280_RECOVERY_COOLDOWN_MS) {
        return;
    }

    if ((uint32_t)(now_ms - s_bmp280_last_service_ms) < BMP280_SERVICE_INTERVAL_MS) {
        return;
    }

    s_bmp280_last_service_ms = now_ms;
    s_bmp280_recovery_pending = 0U;
    BMP280_AttemptRecovery();

    if (s_bmp280_ready == 0U) {
        s_bmp280_recovery_pending = 1U;
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

    if (!s_bmp280_ready) {
        BMP280_HandleReadFailure();
        return 0U;
    }

    if (!BMP280_ReadRegisters(BMP280_REG_PRESS_MSB, raw, sizeof(raw))) {
        BMP280_HandleReadFailure();
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

    BMP280_HandleReadSuccess();
    return 1U;
}

uint8_t BMP280_ReadCompensated(float *pressure_pa, float *temp_c)
{
    int32_t adc_P, adc_T;
    if (!BMP280_ReadRaw(&adc_P, &adc_T)) {
        return 0U;
    }

    float var1, var2, T, p;
    float t_fine;
    
    /* Temperature compensation using 32-bit hardware FPU */
    var1 = (((float)adc_T) / 16384.0f - ((float)s_calib_data.dig_T1) / 1024.0f) * ((float)s_calib_data.dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)s_calib_data.dig_T1) / 8192.0f) *
            (((float)adc_T) / 131072.0f - ((float)s_calib_data.dig_T1) / 8192.0f)) * ((float)s_calib_data.dig_T3);
    t_fine = var1 + var2;
    T = (var1 + var2) / 5120.0f;
    
    if (temp_c != 0) {
        *temp_c = T;
    }

    if (pressure_pa != 0) {
        /* Pressure compensation using 32-bit hardware FPU */
        var1 = (t_fine / 2.0f) - 64000.0f;
        var2 = var1 * var1 * ((float)s_calib_data.dig_P6) / 32768.0f;
        var2 = var2 + var1 * ((float)s_calib_data.dig_P5) * 2.0f;
        var2 = (var2 / 4.0f) + (((float)s_calib_data.dig_P4) * 65536.0f);
        var1 = (((float)s_calib_data.dig_P3) * var1 * var1 / 524288.0f + ((float)s_calib_data.dig_P2) * var1) / 524288.0f;
        var1 = (1.0f + var1 / 32768.0f) * ((float)s_calib_data.dig_P1);
        
        if (var1 == 0.0f) {
            *pressure_pa = 0.0f; /* avoid exception caused by division by zero */
        } else {
            p = 1048576.0f - (float)adc_P;
            p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
            var1 = ((float)s_calib_data.dig_P9) * p * p / 2147483648.0f;
            var2 = p * ((float)s_calib_data.dig_P8) / 32768.0f;
            p = p + (var1 + var2 + ((float)s_calib_data.dig_P7)) / 16.0f;
            
            *pressure_pa = p;
        }
    }
    return 1U;
}
