#include "sensor/magnetometer_sensor.h"

#include "platform/delay.h"
#include "platform/i2c.h"
#include "sensor/sensor_common.h"

#define HMC5883L_ADDR        (0x1E << 1)
#define HMC5883L_SCALE_GAUSS 0.00092f
#define HMC5883L_SCALE_UT    (HMC5883L_SCALE_GAUSS * 100.0f)
#define MAG_LPF_ALPHA        0.08f

void HMC5883L_Init(void)
{
    uint8_t data;

    data = 0x70;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    data = 0x20;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    Delay_us(10U * 1000U);
}

void COMPASS_PROCESS(void)
{
    uint8_t buffer[6];
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDR, 0x03, 1, buffer, 6, 100) == HAL_OK) {
        raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_z = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_y = (int16_t)(buffer[4] << 8 | buffer[5]);

        HMC5883L_RAW_DATA.mag[0] = -(float32_t)raw_x * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[1] = (float32_t)raw_y * HMC5883L_SCALE_UT;
        HMC5883L_RAW_DATA.mag[2] = -(float32_t)raw_z * HMC5883L_SCALE_UT;

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
    }
}
