#include "sensor/magnetometer_sensor.h"
#include "platform/i2c.h"
#include "sensor/mag_calibration.h"
#include "sensor/sensor_common.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

#define HMC5883L_ADDR             0x3CU
#define HMC5883L_REG_CONFIG_A     0x00U
#define HMC5883L_REG_CONFIG_B     0x01U
#define HMC5883L_REG_MODE         0x02U
#define HMC5883L_REG_DATA_X_MSB   0x03U
#define HMC5883L_REG_ID_A         0x0AU
#define HMC5883L_MODE_CONTINUOUS  0x00U
#define HMC5883L_SCALE_UT         0.092f

#define QMC5883L_ADDR             0x1AU
#define QMC5883L_ADDR_7BIT        0x0DU
#define QMC5883L_REG_DATA_X_LSB   0x00U
#define QMC5883L_REG_STATUS       0x06U
#define QMC5883L_REG_CONTROL_1    0x09U
#define QMC5883L_REG_CONTROL_2    0x0AU
#define QMC5883L_REG_SET_RESET    0x0BU
#define QMC5883L_REG_CHIP_ID      0x0DU
#define QMC5883L_CTRL_200HZ_8G_CONT 0x1DU
#define QMC5883L_CTRL_SOFT_RESET  0x80U
#define QMC5883L_SET_RESET_DEFAULT 0x01U
#define QMC5883L_STATUS_DRDY      0x01U

#define MAG_I2C_INIT_TIMEOUT_MS   50U
#define MAG_I2C_READ_TIMEOUT_MS   10U
#define MAG_FAIL_STREAK_LIMIT     3U
#define MAG_RECOVERY_INTERVAL_MS  500U

#define MAG_SWAP_XY               0
#define MAG_SIGN_X                1.0f
#define MAG_SIGN_Y                -1.0f
#define MAG_SIGN_Z                1.0f

static uint8_t s_magnetometer_type = MAGNETOMETER_TYPE_NONE;
static uint8_t s_magnetometer_ready = 0U;
static uint8_t s_magnetometer_last_read_ok = 0U;
static uint8_t s_magnetometer_addr_7bit = 0U;
static uint8_t s_magnetometer_chip_id = 0U;
static uint8_t s_magnetometer_id[3] = {0U, 0U, 0U};
static uint8_t s_magnetometer_fail_streak = 0U;
static uint8_t s_magnetometer_recovery_pending = 0U;
static uint32_t s_last_recovery_tick = 0U;

static uint8_t Magnetometer_WriteRegisterWithTimeout(uint16_t address, uint8_t reg, uint8_t data, uint32_t timeout)
{
    return I2C_Mem_WriteRecover(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1U, timeout);
}

static uint8_t Magnetometer_ReadRegistersWithTimeout(uint16_t address, uint8_t reg, uint8_t *data, uint16_t len, uint32_t timeout)
{
    return I2C_Mem_ReadRecover(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
}

static uint8_t Magnetometer_ReadRegisters(uint16_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
    return Magnetometer_ReadRegistersWithTimeout(address, reg, data, len, MAG_I2C_READ_TIMEOUT_MS);
}

static void Delay_ms_blocking(uint32_t ms)
{
    HAL_Delay(ms);
}

static void Magnetometer_ResetState(void)
{
    s_magnetometer_type = MAGNETOMETER_TYPE_NONE;
    s_magnetometer_ready = 0U;
    s_magnetometer_last_read_ok = 0U;
    s_magnetometer_addr_7bit = 0U;
    s_magnetometer_chip_id = 0U;
    s_magnetometer_id[0] = 0U;
    s_magnetometer_id[1] = 0U;
    s_magnetometer_id[2] = 0U;
    s_magnetometer_fail_streak = 0U;
    s_magnetometer_recovery_pending = 0U;
    mag_lpf_inited = 0U;
}

static uint8_t Magnetometer_TryInitHmc5883l(void)
{
    uint8_t id[3];

    if (Magnetometer_ReadRegistersWithTimeout(HMC5883L_ADDR, HMC5883L_REG_ID_A, id, 3U, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    if ((id[0] != 'H') || (id[1] != '4') || (id[2] != '3')) {
        return 0U;
    }

    if (Magnetometer_WriteRegisterWithTimeout(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x70, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegisterWithTimeout(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0x20, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegisterWithTimeout(HMC5883L_ADDR, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    s_magnetometer_type = MAGNETOMETER_TYPE_HMC5883L;
    s_magnetometer_ready = 1U;
    s_magnetometer_addr_7bit = 0x1EU;
    s_magnetometer_chip_id = id[0];
    s_magnetometer_id[0] = id[0];
    s_magnetometer_id[1] = id[1];
    s_magnetometer_id[2] = id[2];
    return 1U;
}

static uint8_t Magnetometer_TryInitQmc5883l(void)
{
    uint8_t chip_id = 0U;
    uint8_t status = 0U;

    if (Magnetometer_ReadRegistersWithTimeout(QMC5883L_ADDR, QMC5883L_REG_CHIP_ID, &chip_id, 1U, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    if (Magnetometer_WriteRegisterWithTimeout(QMC5883L_ADDR, QMC5883L_REG_CONTROL_2, QMC5883L_CTRL_SOFT_RESET, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }
    Delay_ms_blocking(5U);

    if (Magnetometer_WriteRegisterWithTimeout(QMC5883L_ADDR, QMC5883L_REG_SET_RESET, QMC5883L_SET_RESET_DEFAULT, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegisterWithTimeout(QMC5883L_ADDR, QMC5883L_REG_CONTROL_1, QMC5883L_CTRL_200HZ_8G_CONT, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }
    Delay_ms_blocking(20U);

    if (Magnetometer_ReadRegistersWithTimeout(QMC5883L_ADDR, QMC5883L_REG_STATUS, &status, 1U, MAG_I2C_INIT_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    s_magnetometer_type = MAGNETOMETER_TYPE_QMC5883L;
    s_magnetometer_ready = 1U;
    s_magnetometer_addr_7bit = QMC5883L_ADDR_7BIT;
    s_magnetometer_chip_id = chip_id;
    return 1U;
}

static void Magnetometer_StoreCalibratedData(MAG_RAW_DATA_t *raw, MAG_DATA_t *cal)
{
    MagCal_Update(&MagCal, raw, cal);

    if (MagCal.state == MAG_CAL_DONE) {
        if (!mag_lpf_inited) {
            mag_filtered[0] = cal->mag_uT[0];
            mag_filtered[1] = cal->mag_uT[1];
            mag_filtered[2] = cal->mag_uT[2];
            mag_lpf_inited = 1;
        } else {
            mag_filtered[0] = mag_filtered[0] * 0.9f + cal->mag_uT[0] * 0.1f;
            mag_filtered[1] = mag_filtered[1] * 0.9f + cal->mag_uT[1] * 0.1f;
            mag_filtered[2] = mag_filtered[2] * 0.9f + cal->mag_uT[2] * 0.1f;
        }
    }
}

static void Magnetometer_HandleReadFailure(void)
{
    s_magnetometer_last_read_ok = 0U;
    s_magnetometer_fail_streak++;
    if (s_magnetometer_fail_streak >= MAG_FAIL_STREAK_LIMIT) {
        s_magnetometer_ready = 0U;
        s_magnetometer_recovery_pending = 1U;
    }
}

static void Magnetometer_HandleReadSuccess(void)
{
    s_magnetometer_last_read_ok = 1U;
    s_magnetometer_fail_streak = 0U;
}

void Magnetometer_Init(void)
{
    Magnetometer_ResetState();
    if (Magnetometer_TryInitQmc5883l() != 0U) return;
    if (Magnetometer_TryInitHmc5883l() != 0U) return;
}

void Magnetometer_Service(uint32_t now_ms)
{
    if (s_magnetometer_recovery_pending != 0U) {
        if ((uint32_t)(now_ms - s_last_recovery_tick) >= MAG_RECOVERY_INTERVAL_MS) {
            s_last_recovery_tick = now_ms;
            Magnetometer_Init();
        }
    }
}

void COMPASS_PROCESS(void)
{
    uint8_t buffer[7];
    uint8_t status = 0;
    int16_t raw_x = 0, raw_y = 0, raw_z = 0;
    float32_t mx = 0, my = 0, mz = 0;

    /* Always allow state machine to run even if sensor has failures, 
     * but we still need s_magnetometer_ready for actual data reading. */
    if (!s_magnetometer_ready && MagCal.state == MAG_CAL_DONE) {
        Magnetometer_HandleReadFailure();
        return;
    }

    /* Transition START -> COLLECTING does not need sensor data */
    if (MagCal.state == MAG_CAL_START) {
        MagCal_Update(&MagCal, &MAG_RAW_DATA_INST, &MAG_DATA_INST);
    }

    if (s_magnetometer_type == MAGNETOMETER_TYPE_HMC5883L) {
        if (Magnetometer_ReadRegisters(HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB, buffer, 6U) == 0U) {
            Magnetometer_HandleReadFailure();
            return;
        }
        raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_z = (int16_t)((buffer[2] << 8) | buffer[3]);
        raw_y = (int16_t)((buffer[4] << 8) | buffer[5]);
        mx = (float32_t)raw_x * HMC5883L_SCALE_UT;
        mz = (float32_t)raw_z * HMC5883L_SCALE_UT;
        my = (float32_t)raw_y * HMC5883L_SCALE_UT;
    } else if (s_magnetometer_type == MAGNETOMETER_TYPE_QMC5883L) {
        /* Some QMC5883L clones have unreliable DRDY bits. 
         * We skip the DRDY check and read registers directly. */
        if (Magnetometer_ReadRegisters(QMC5883L_ADDR, QMC5883L_REG_DATA_X_LSB, buffer, 6U) == 0U) {
            Magnetometer_HandleReadFailure();
            return;
        }
        raw_x = (int16_t)(buffer[0] | (buffer[1] << 8));
        raw_y = (int16_t)(buffer[2] | (buffer[3] << 8));
        raw_z = (int16_t)(buffer[4] | (buffer[5] << 8));
        mx = (float32_t)raw_x;
        my = (float32_t)raw_y;
        mz = (float32_t)raw_z;
    } else {
        return;
    }

#if MAG_SWAP_XY
    MAG_RAW_DATA_INST.mag[0] = (float32_t)my * MAG_SIGN_Y;
    MAG_RAW_DATA_INST.mag[1] = (float32_t)mx * MAG_SIGN_X;
#else
    MAG_RAW_DATA_INST.mag[0] = (float32_t)mx * MAG_SIGN_X;
    MAG_RAW_DATA_INST.mag[1] = (float32_t)my * MAG_SIGN_Y;
#endif
    MAG_RAW_DATA_INST.mag[2] = (float32_t)mz * MAG_SIGN_Z;

    Magnetometer_StoreCalibratedData(&MAG_RAW_DATA_INST, &MAG_DATA_INST);
    Magnetometer_HandleReadSuccess();
}

uint8_t Magnetometer_IsReady(void)
{
    return s_magnetometer_ready;
}

uint8_t Magnetometer_GetLastReadOk(void)
{
    return s_magnetometer_last_read_ok;
}

uint8_t Magnetometer_GetChipId(void)
{
    return s_magnetometer_chip_id;
}

uint8_t Magnetometer_GetType(void)
{
    return s_magnetometer_type;
}

uint8_t Magnetometer_GetAddress7Bit(void)
{
    return s_magnetometer_addr_7bit;
}

void Magnetometer_GetId(uint8_t id_out[3])
{
    id_out[0] = s_magnetometer_id[0];
    id_out[1] = s_magnetometer_id[1];
    id_out[2] = s_magnetometer_id[2];
}
