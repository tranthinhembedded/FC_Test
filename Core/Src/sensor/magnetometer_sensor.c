#include "sensor/magnetometer_sensor.h"

#include "platform/delay.h"
#include "platform/i2c.h"
#include "sensor/sensor_common.h"

#define MAG_I2C_TIMEOUT_MS           5U

#define HMC5883L_ADDR_7BIT           0x1EU
#define HMC5883L_ADDR                (HMC5883L_ADDR_7BIT << 1)
#define HMC5883L_REG_ID_A            0x0AU
#define HMC5883L_REG_DATA_X_MSB      0x03U
#define HMC5883L_SCALE_GAUSS         0.00092f
#define HMC5883L_SCALE_UT            (HMC5883L_SCALE_GAUSS * 100.0f)

#define QMC5883L_ADDR_7BIT           0x0DU
#define QMC5883L_ADDR                (QMC5883L_ADDR_7BIT << 1)
#define QMC5883L_REG_DATA_X_LSB      0x00U
#define QMC5883L_REG_STATUS          0x06U
#define QMC5883L_REG_CONTROL_1       0x09U
#define QMC5883L_REG_CONTROL_2       0x0AU
#define QMC5883L_REG_SET_RESET       0x0BU
#define QMC5883L_REG_CHIP_ID         0x0DU
#define QMC5883L_CHIP_ID             0xFFU
#define QMC5883L_STATUS_DRDY         0x01U
#define QMC5883L_CTRL_200HZ_8G_CONT  0x1DU
#define QMC5883L_CTRL_SOFT_RESET     0x80U
#define QMC5883L_SET_RESET_DEFAULT   0x01U
#define QMC5883L_SCALE_UT            (100.0f / 3000.0f)

#define MAG_LPF_ALPHA                0.08f

/*
 * User confirmed: Magnetometer is rotated 180 degrees CCW compared to the original code.
 * Therefore, NO X/Y swap is needed (only 90/270 degree rotations require that).
 * Instead, we just invert the X and Y outputs of the original code.
 */
#define MAG_SWAP_XY                  0

/* Cân chỉnh chiều của la bàn.
 * Original Code: X = -raw_x, Y = raw_y, Z = -raw_z
 * 180 Deg Rotate: Invert X and Y -> X = raw_x, Y = -raw_y, Z = -raw_z
 */
#define MAG_SIGN_X                   (1.0f)
#define MAG_SIGN_Y                   (-1.0f)
#define MAG_SIGN_Z                   (-1.0f)

static uint8_t s_magnetometer_type = MAGNETOMETER_TYPE_NONE;
static uint8_t s_magnetometer_ready = 0U;
static uint8_t s_magnetometer_last_read_ok = 0U;
static uint8_t s_magnetometer_addr_7bit = 0U;
static uint8_t s_magnetometer_chip_id = 0U;
static uint8_t s_magnetometer_id[3] = {0U, 0U, 0U};

static uint8_t Magnetometer_ReadRegisters(uint16_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, data, len, MAG_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t Magnetometer_WriteRegister(uint16_t address, uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, MAG_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
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
}

static uint8_t Magnetometer_TryInitHmc5883l(void)
{
    uint8_t id[3] = {0U, 0U, 0U};

    if (Magnetometer_ReadRegisters(HMC5883L_ADDR, HMC5883L_REG_ID_A, id, sizeof(id)) == 0U) {
        return 0U;
    }

    if ((id[0] != 'H') || (id[1] != '4') || (id[2] != '3')) {
        return 0U;
    }

    if (Magnetometer_WriteRegister(HMC5883L_ADDR, 0x00U, 0x70U) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegister(HMC5883L_ADDR, 0x01U, 0x20U) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegister(HMC5883L_ADDR, 0x02U, 0x00U) == 0U) {
        return 0U;
    }

    Delay_ms_blocking(10U);

    s_magnetometer_type = MAGNETOMETER_TYPE_HMC5883L;
    s_magnetometer_ready = 1U;
    s_magnetometer_addr_7bit = HMC5883L_ADDR_7BIT;
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

    if (Magnetometer_ReadRegisters(QMC5883L_ADDR, QMC5883L_REG_CHIP_ID, &chip_id, 1U) == 0U) {
        return 0U;
    }

    if (Magnetometer_WriteRegister(QMC5883L_ADDR, QMC5883L_REG_CONTROL_2, QMC5883L_CTRL_SOFT_RESET) == 0U) {
        return 0U;
    }
    Delay_ms_blocking(1U);

    if (Magnetometer_WriteRegister(QMC5883L_ADDR, QMC5883L_REG_SET_RESET, QMC5883L_SET_RESET_DEFAULT) == 0U) {
        return 0U;
    }
    if (Magnetometer_WriteRegister(QMC5883L_ADDR, QMC5883L_REG_CONTROL_1, QMC5883L_CTRL_200HZ_8G_CONT) == 0U) {
        return 0U;
    }
    Delay_ms_blocking(10U);

    /* A successful status read after configuration is enough to treat the device as QMC-compatible.
     * Official QST parts return 0xFF in the chip-ID register, but some modules expose compatible variants.
     */
    if (Magnetometer_ReadRegisters(QMC5883L_ADDR, QMC5883L_REG_STATUS, &status, 1U) == 0U) {
        return 0U;
    }

    s_magnetometer_type = MAGNETOMETER_TYPE_QMC5883L;
    s_magnetometer_ready = 1U;
    s_magnetometer_addr_7bit = QMC5883L_ADDR_7BIT;
    s_magnetometer_chip_id = chip_id;
    s_magnetometer_id[0] = chip_id;
    s_magnetometer_id[1] = 0U;
    s_magnetometer_id[2] = 0U;
    return 1U;
}

static void Magnetometer_StoreCalibratedData(float32_t mx, float32_t my, float32_t mz)
{
    HMC5883L_RAW_DATA.mag[0] = MAG_SIGN_X * mx;
    HMC5883L_RAW_DATA.mag[1] = MAG_SIGN_Y * my;
    HMC5883L_RAW_DATA.mag[2] = MAG_SIGN_Z * mz;

    MagCal_Update(&MagCal, &HMC5883L_RAW_DATA, &HMC5883L_DATA);

    if (MagCal.state == MAG_CAL_DONE) {
        if (!mag_lpf_inited) {
            mag_filtered[0] = HMC5883L_DATA.mag_uT[0];
            mag_filtered[1] = HMC5883L_DATA.mag_uT[1];
            mag_filtered[2] = HMC5883L_DATA.mag_uT[2];
            mag_lpf_inited = 1U;
        } else {
            mag_filtered[0] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[0] - mag_filtered[0]);
            mag_filtered[1] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[1] - mag_filtered[1]);
            mag_filtered[2] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[2] - mag_filtered[2]);
        }
        HMC5883L_DATA.mag_uT[0] = mag_filtered[0];
        HMC5883L_DATA.mag_uT[1] = mag_filtered[1];
        HMC5883L_DATA.mag_uT[2] = mag_filtered[2];
    }
}

void HMC5883L_Init(void)
{
    Magnetometer_ResetState();

    if (Magnetometer_TryInitHmc5883l() != 0U) {
        return;
    }

    (void)Magnetometer_TryInitQmc5883l();
}

uint8_t HMC5883L_IsReady(void)
{
    return s_magnetometer_ready;
}

void HMC5883L_GetId(uint8_t id_out[3])
{
    if (id_out == 0) {
        return;
    }

    id_out[0] = s_magnetometer_id[0];
    id_out[1] = s_magnetometer_id[1];
    id_out[2] = s_magnetometer_id[2];
}

uint8_t HMC5883L_GetLastReadOk(void)
{
    return s_magnetometer_last_read_ok;
}

uint8_t Magnetometer_GetType(void)
{
    return s_magnetometer_type;
}

uint8_t Magnetometer_GetAddress7Bit(void)
{
    return s_magnetometer_addr_7bit;
}

uint8_t Magnetometer_GetChipId(void)
{
    return s_magnetometer_chip_id;
}

void COMPASS_PROCESS(void)
{
    uint8_t buffer[6];
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float32_t mx;
    float32_t my;
    float32_t mz;

    if (!s_magnetometer_ready) {
        s_magnetometer_last_read_ok = 0U;
        return;
    }

    if (s_magnetometer_type == MAGNETOMETER_TYPE_HMC5883L) {
        if (Magnetometer_ReadRegisters(HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB, buffer, sizeof(buffer)) == 0U) {
            s_magnetometer_last_read_ok = 0U;
            return;
        }

        raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_z = (int16_t)((buffer[2] << 8) | buffer[3]);
        raw_y = (int16_t)((buffer[4] << 8) | buffer[5]);

#if MAG_SWAP_XY
        mx = (float32_t)raw_y;
        my = (float32_t)raw_x;
#else
        mx = (float32_t)raw_x;
        my = (float32_t)raw_y;
#endif
        mz = (float32_t)raw_z;

        Magnetometer_StoreCalibratedData(mx * HMC5883L_SCALE_UT, my * HMC5883L_SCALE_UT, mz * HMC5883L_SCALE_UT);
        s_magnetometer_last_read_ok = 1U;
        return;
    }

    if (s_magnetometer_type == MAGNETOMETER_TYPE_QMC5883L) {
        uint8_t status = 0U;

        if (Magnetometer_ReadRegisters(QMC5883L_ADDR, QMC5883L_REG_STATUS, &status, 1U) == 0U) {
            s_magnetometer_last_read_ok = 0U;
            return;
        }

        if ((status & QMC5883L_STATUS_DRDY) == 0U) {
            s_magnetometer_last_read_ok = 0U;
            return;
        }

        if (Magnetometer_ReadRegisters(QMC5883L_ADDR, QMC5883L_REG_DATA_X_LSB, buffer, sizeof(buffer)) == 0U) {
            s_magnetometer_last_read_ok = 0U;
            return;
        }

        raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

#if MAG_SWAP_XY
        mx = (float32_t)raw_y;
        my = (float32_t)raw_x;
#else
        mx = (float32_t)raw_x;
        my = (float32_t)raw_y;
#endif
        mz = (float32_t)raw_z;

        Magnetometer_StoreCalibratedData(mx * QMC5883L_SCALE_UT, my * QMC5883L_SCALE_UT, mz * QMC5883L_SCALE_UT);
        s_magnetometer_last_read_ok = 1U;
        return;
    }

    s_magnetometer_last_read_ok = 0U;
}
