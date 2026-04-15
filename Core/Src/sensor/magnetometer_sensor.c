#include "sensor/magnetometer_sensor.h"

#include "platform/delay.h"
#include "platform/i2c.h"
#include "sensor/sensor_common.h"

#define HMC5883L_ADDR        (0x1E << 1)
#define HMC5883L_SCALE_GAUSS 0.00092f
#define HMC5883L_SCALE_UT    (HMC5883L_SCALE_GAUSS * 100.0f)
#define MAG_LPF_ALPHA        0.08f
#define HMC5883L_I2C_TIMEOUT_MS 5U

/* 
 * User confirmed: Magnetometer is rotated 180 degrees CCW compared to the original code.
 * Therefore, NO X/Y swap is needed (only 90/270 degree rotations require that).
 * Instead, we just invert the X and Y outputs of the original code.
 */
#define MAG_SWAP_XY    0

/* Cân chỉnh chiều của la bàn 
 * Original Code: X = -raw_x, Y = raw_y, Z = -raw_z
 * 180 Deg Rotate: Invert X and Y -> X = raw_x, Y = -raw_y, Z = -raw_z
 */
#define MAG_SIGN_X  (1.0f)
#define MAG_SIGN_Y  (-1.0f)
#define MAG_SIGN_Z  (-1.0f)

static uint8_t s_hmc5883l_ready = 0U;
static uint8_t s_hmc5883l_last_read_ok = 0U;
static uint8_t s_hmc5883l_id[3] = {0U, 0U, 0U};

void HMC5883L_Init(void)
{
    uint8_t id[3] = {0U, 0U, 0U};
    uint8_t data;

    s_hmc5883l_ready = 0U;
    s_hmc5883l_last_read_ok = 0U;
    s_hmc5883l_id[0] = 0U;
    s_hmc5883l_id[1] = 0U;
    s_hmc5883l_id[2] = 0U;

    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x0A, 1, id, 3, HMC5883L_I2C_TIMEOUT_MS) == HAL_OK) {
        s_hmc5883l_id[0] = id[0];
        s_hmc5883l_id[1] = id[1];
        s_hmc5883l_id[2] = id[2];
        if (id[0] == 'H' && id[1] == '4' && id[2] == '3') {
            s_hmc5883l_ready = 1U;
        }
    }

    data = 0x70;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, HMC5883L_I2C_TIMEOUT_MS);
    data = 0x20;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x01, 1, &data, 1, HMC5883L_I2C_TIMEOUT_MS);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, HMC5883L_I2C_TIMEOUT_MS);
    Delay_us(10U * 1000U);
}

uint8_t HMC5883L_IsReady(void)
{
    return s_hmc5883l_ready;
}

void HMC5883L_GetId(uint8_t id_out[3])
{
    if (id_out == 0) {
        return;
    }

    id_out[0] = s_hmc5883l_id[0];
    id_out[1] = s_hmc5883l_id[1];
    id_out[2] = s_hmc5883l_id[2];
}

uint8_t HMC5883L_GetLastReadOk(void)
{
    return s_hmc5883l_last_read_ok;
}

void COMPASS_PROCESS(void)
{
    uint8_t buffer[6];
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (!s_hmc5883l_ready) {
        s_hmc5883l_last_read_ok = 0U;
        return;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, HMC5883L_I2C_TIMEOUT_MS) == HAL_OK) {
        s_hmc5883l_last_read_ok = 1U;
        float32_t mx, my, mz;
        raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_z = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_y = (int16_t)(buffer[4] << 8 | buffer[5]);

#if MAG_SWAP_XY
        mx = (float32_t)raw_y;
        my = (float32_t)raw_x;
#else
        mx = (float32_t)raw_x;
        my = (float32_t)raw_y;
#endif
        mz = (float32_t)raw_z;

        HMC5883L_RAW_DATA.mag[0] = MAG_SIGN_X * mx * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[1] = MAG_SIGN_Y * my * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[2] = MAG_SIGN_Z * mz * HMC5883L_SCALE_UT;

        MagCal_Update(&MagCal, &HMC5883L_RAW_DATA, &HMC5883L_DATA);

        if (MagCal.state == MAG_CAL_DONE) {
            if (!mag_lpf_inited) {
                mag_filtered[0] = HMC5883L_DATA.mag_uT[0];
                mag_filtered[1] = HMC5883L_DATA.mag_uT[1];
                mag_filtered[2] = HMC5883L_DATA.mag_uT[2];
                mag_lpf_inited = 1;
            } else {
                mag_filtered[0] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[0] - mag_filtered[0]);
                mag_filtered[1] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[1] - mag_filtered[1]);
                mag_filtered[2] += MAG_LPF_ALPHA * (HMC5883L_DATA.mag_uT[2] - mag_filtered[2]);
            }
            HMC5883L_DATA.mag_uT[0] = mag_filtered[0];
            HMC5883L_DATA.mag_uT[1] = mag_filtered[1];
            HMC5883L_DATA.mag_uT[2] = mag_filtered[2];
        }
    } else {
        s_hmc5883l_last_read_ok = 0U;
    }
}
