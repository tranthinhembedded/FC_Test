#include "sensor/imu_config.h"

#include "platform/delay.h"
#include "platform/spi.h"
#include "sensor/sensor_common.h"

#define ICM20602_REG_SMPLRT_DIV     0x19U
#define ICM20602_REG_CONFIG         0x1AU
#define ICM20602_REG_GYRO_CONFIG    0x1BU
#define ICM20602_REG_ACCEL_CONFIG   0x1CU
#define ICM20602_REG_ACCEL_CONFIG2  0x1DU
#define ICM20602_REG_ACCEL_XOUT_H   0x3BU
#define ICM20602_REG_USER_CTRL      0x6AU
#define ICM20602_REG_PWR_MGMT_1     0x6BU
#define ICM20602_REG_PWR_MGMT_2     0x6CU
#define ICM20602_REG_I2C_IF         0x70U
#define ICM20602_REG_WHO_AM_I       0x75U

#define ICM20602_READ_FLAG          0x80U
#define ICM20602_WHO_AM_I_VALUE     0x12U
#define ICM20602_GYRO_SENSITIVITY   131.0f
#define ICM20602_ACCEL_SENSITIVITY  16384.0f
#define ICM20602_GRAVITY_EARTH      9.80665f
#define ICM20602_CALIB_SAMPLES      1000U
#define ICM20602_SPI_TIMEOUT_MS     20U
#define ICM20602_BURST_LENGTH       14U

/*
 * Adjust these signs if the breakout is mounted with a different
 * orientation on the airframe.
 */
#define ICM20602_ACCEL_SIGN_X       (1.0f)
#define ICM20602_ACCEL_SIGN_Y       (-1.0f)
#define ICM20602_ACCEL_SIGN_Z       (-1.0f)
#define ICM20602_GYRO_SIGN_X        (1.0f)
#define ICM20602_GYRO_SIGN_Y        (-1.0f)
#define ICM20602_GYRO_SIGN_Z        (-1.0f)

static uint8_t s_icm20602_ready = 0U;

static void ICM20602_Select(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void ICM20602_Deselect(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ICM20602_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {reg & (uint8_t)~ICM20602_READ_FLAG, value};
    HAL_StatusTypeDef status;

    ICM20602_Select();
    status = HAL_SPI_Transmit(&hspi1, tx_buf, 2U, ICM20602_SPI_TIMEOUT_MS);
    ICM20602_Deselect();

    return status;
}

static HAL_StatusTypeDef ICM20602_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[1U + ICM20602_BURST_LENGTH] = {0};
    uint8_t rx_buf[1U + ICM20602_BURST_LENGTH] = {0};
    HAL_StatusTypeDef status;
    uint16_t i;

    if (len == 0U || len > ICM20602_BURST_LENGTH) {
        return HAL_ERROR;
    }

    tx_buf[0] = reg | ICM20602_READ_FLAG;

    ICM20602_Select();
    status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, (uint16_t)(len + 1U), ICM20602_SPI_TIMEOUT_MS);
    ICM20602_Deselect();

    if (status != HAL_OK) {
        return status;
    }

    for (i = 0U; i < len; i++) {
        data[i] = rx_buf[i + 1U];
    }

    return HAL_OK;
}

static HAL_StatusTypeDef ICM20602_ReadRegister(uint8_t reg, uint8_t *value)
{
    return ICM20602_ReadRegisters(reg, value, 1U);
}

static void ICM20602_ParseMotionFrame(uint8_t *buffer)
{
    float32_t acc_temp[3];
    float32_t gyro_temp[3];
    int i;
    static uint8_t acc_lpf_inited = 0U;
    static uint8_t gyro_lpf_inited = 0U;

    raw_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
    raw_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
    raw_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
    raw_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
    raw_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
    raw_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

    if (is_calibrated) {
        acc_temp[0] = (float32_t)raw_acc[0] - accel_bias[0];
        acc_temp[1] = (float32_t)raw_acc[1] - accel_bias[1];
        acc_temp[2] = (float32_t)raw_acc[2] - accel_bias[2];
        gyro_temp[0] = (float32_t)raw_gyro[0] - gyro_bias[0];
        gyro_temp[1] = (float32_t)raw_gyro[1] - gyro_bias[1];
        gyro_temp[2] = (float32_t)raw_gyro[2] - gyro_bias[2];
    } else {
        acc_temp[0] = raw_acc[0];
        acc_temp[1] = raw_acc[1];
        acc_temp[2] = raw_acc[2];
        gyro_temp[0] = raw_gyro[0];
        gyro_temp[1] = raw_gyro[1];
        gyro_temp[2] = raw_gyro[2];
    }

    acc_phys[0] = ICM20602_ACCEL_SIGN_X * (acc_temp[0] / ICM20602_ACCEL_SENSITIVITY) * ICM20602_GRAVITY_EARTH;
    acc_phys[1] = ICM20602_ACCEL_SIGN_Y * (acc_temp[1] / ICM20602_ACCEL_SENSITIVITY) * ICM20602_GRAVITY_EARTH;
    acc_phys[2] = ICM20602_ACCEL_SIGN_Z * (acc_temp[2] / ICM20602_ACCEL_SENSITIVITY) * ICM20602_GRAVITY_EARTH;

    gyro_phys[0] = ICM20602_GYRO_SIGN_X * (gyro_temp[0] / ICM20602_GYRO_SENSITIVITY) * DEG_TO_RAD;
    gyro_phys[1] = ICM20602_GYRO_SIGN_Y * (gyro_temp[1] / ICM20602_GYRO_SENSITIVITY) * DEG_TO_RAD;
    gyro_phys[2] = ICM20602_GYRO_SIGN_Z * (gyro_temp[2] / ICM20602_GYRO_SENSITIVITY) * DEG_TO_RAD;

    if (!acc_lpf_inited) {
        acc_filtered[0] = acc_phys[0];
        acc_filtered[1] = acc_phys[1];
        acc_filtered[2] = acc_phys[2];
        acc_lpf_inited = 1U;
    } else {
        for (i = 0; i < 3; i++) {
            acc_filtered[i] += alpha_acc_soft * (acc_phys[i] - acc_filtered[i]);
        }
    }

    if (!gyro_lpf_inited) {
        gyro_final[0] = gyro_phys[0];
        gyro_final[1] = gyro_phys[1];
        gyro_final[2] = gyro_phys[2];
        gyro_lpf_inited = 1U;
    } else {
        for (i = 0; i < 3; i++) {
            gyro_final[i] += alpha_gyro * (gyro_phys[i] - gyro_final[i]);
        }
    }

    MPU6500_DATA.acc[0] = acc_filtered[0];
    MPU6500_DATA.acc[1] = acc_filtered[1];
    MPU6500_DATA.acc[2] = acc_filtered[2];
    MPU6500_DATA.w[0] = gyro_final[0];
    MPU6500_DATA.w[1] = gyro_final[1];
    MPU6500_DATA.w[2] = gyro_final[2];
}

#if 0
/*
 * Legacy MPU6050 I2C implementation kept only as commented reference while
 * migrating this project to the SPI ICM-20602 breakout.
 */

#define MPU6050_ADDR      (0x68 << 1)
#define GYRO_SENSITIVITY  65.5f
#define ACCEL_SENSITIVITY 4096.0f
#define GRAVITY_EARTH     9.80665f

void MPU6050_Init(void)
{
    uint8_t data;
    uint8_t bypass_en;

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
    data = 0x06;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);

    bypass_en = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x37, 1, &bypass_en, 1, 100);
}

void MPU6050_Calibrate(void)
{
    uint8_t buffer[14];
    int32_t sum_acc[3] = {0};
    int32_t sum_gyro[3] = {0};
    int16_t ra_acc[3];
    int16_t ra_gyro[3];
    int sample_count = 1000;
    int i;

    for (i = 0; i < sample_count; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
            ra_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
            ra_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
            ra_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
            ra_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
            ra_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
            ra_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

            sum_acc[0] += ra_acc[0];
            sum_acc[1] += ra_acc[1];
            sum_acc[2] += ra_acc[2];
            sum_gyro[0] += ra_gyro[0];
            sum_gyro[1] += ra_gyro[1];
            sum_gyro[2] += ra_gyro[2];
        }
        Delay_us(2U * 1000U);
    }

    gyro_bias[0] = (float)sum_gyro[0] / sample_count;
    gyro_bias[1] = (float)sum_gyro[1] / sample_count;
    gyro_bias[2] = (float)sum_gyro[2] / sample_count;
    accel_bias[0] = (float)sum_acc[0] / sample_count;
    accel_bias[1] = (float)sum_acc[1] / sample_count;
    accel_bias[2] = ((float)sum_acc[2] / sample_count) - 4096.0f;
    is_calibrated = 1;
}

void IMU_PROCESS(void)
{
    uint8_t buffer[14];

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 14, 10) == HAL_OK) {
        float32_t acc_temp[3];
        float32_t gyro_temp[3];
        int i;
        static uint8_t acc_lpf_inited = 0;
        static uint8_t gyro_lpf_inited = 0;

        raw_acc[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
        raw_acc[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
        raw_acc[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
        raw_gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
        raw_gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
        raw_gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);

        if (is_calibrated) {
            acc_temp[0] = (float32_t)raw_acc[0] - accel_bias[0];
            acc_temp[1] = (float32_t)raw_acc[1] - accel_bias[1];
            acc_temp[2] = (float32_t)raw_acc[2] - accel_bias[2];
            gyro_temp[0] = (float32_t)raw_gyro[0] - gyro_bias[0];
            gyro_temp[1] = (float32_t)raw_gyro[1] - gyro_bias[1];
            gyro_temp[2] = (float32_t)raw_gyro[2] - gyro_bias[2];
        } else {
            acc_temp[0] = raw_acc[0];
            acc_temp[1] = raw_acc[1];
            acc_temp[2] = raw_acc[2];
            gyro_temp[0] = raw_gyro[0];
            gyro_temp[1] = raw_gyro[1];
            gyro_temp[2] = raw_gyro[2];
        }

        acc_phys[0] = (acc_temp[0] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[1] = -(acc_temp[1] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        acc_phys[2] = -(acc_temp[2] / ACCEL_SENSITIVITY) * GRAVITY_EARTH;
        gyro_phys[0] = (gyro_temp[0] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[1] = -(gyro_temp[1] / GYRO_SENSITIVITY) * DEG_TO_RAD;
        gyro_phys[2] = -(gyro_temp[2] / GYRO_SENSITIVITY) * DEG_TO_RAD;

        if (!acc_lpf_inited) {
            acc_filtered[0] = acc_phys[0];
            acc_filtered[1] = acc_phys[1];
            acc_filtered[2] = acc_phys[2];
            acc_lpf_inited = 1;
        } else {
            for (i = 0; i < 3; i++) {
                acc_filtered[i] += alpha_acc_soft * (acc_phys[i] - acc_filtered[i]);
            }
        }

        if (!gyro_lpf_inited) {
            gyro_final[0] = gyro_phys[0];
            gyro_final[1] = gyro_phys[1];
            gyro_final[2] = gyro_phys[2];
            gyro_lpf_inited = 1;
        } else {
            for (i = 0; i < 3; i++) {
                gyro_final[i] += alpha_gyro * (gyro_phys[i] - gyro_final[i]);
            }
        }

        MPU6500_DATA.acc[0] = acc_filtered[0];
        MPU6500_DATA.acc[1] = acc_filtered[1];
        MPU6500_DATA.acc[2] = acc_filtered[2];
        MPU6500_DATA.w[0] = gyro_final[0];
        MPU6500_DATA.w[1] = gyro_final[1];
        MPU6500_DATA.w[2] = gyro_final[2];
    }
}
#endif

void ICM20602_Init(void)
{
    uint8_t who_am_i = 0U;

    s_icm20602_ready = 0U;
    is_calibrated = 0U;

    ICM20602_Deselect();
    Delay_ms_blocking(10U);

    if (ICM20602_WriteRegister(ICM20602_REG_PWR_MGMT_1, 0x80U) != HAL_OK) {
        return;
    }
    Delay_ms_blocking(100U);

    if (ICM20602_WriteRegister(ICM20602_REG_PWR_MGMT_1, 0x01U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_PWR_MGMT_2, 0x00U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_USER_CTRL, 0x01U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_I2C_IF, 0x40U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_CONFIG, 0x05U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_SMPLRT_DIV, 0x00U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_GYRO_CONFIG, 0x00U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_ACCEL_CONFIG, 0x00U) != HAL_OK) {
        return;
    }
    if (ICM20602_WriteRegister(ICM20602_REG_ACCEL_CONFIG2, 0x05U) != HAL_OK) {
        return;
    }

    Delay_ms_blocking(20U);

    if (ICM20602_ReadRegister(ICM20602_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return;
    }

    if (who_am_i == ICM20602_WHO_AM_I_VALUE) {
        s_icm20602_ready = 1U;
    }
}

void ICM20602_Calibrate(void)
{
    uint8_t buffer[ICM20602_BURST_LENGTH];
    int32_t sum_gyro[3] = {0};
    uint32_t valid_samples = 0U;
    uint32_t i;

    if (!s_icm20602_ready) {
        return;
    }

    for (i = 0U; i < ICM20602_CALIB_SAMPLES; i++) {
        if (ICM20602_ReadRegisters(ICM20602_REG_ACCEL_XOUT_H, buffer, ICM20602_BURST_LENGTH) == HAL_OK) {
            sum_gyro[0] += (int16_t)(buffer[8] << 8 | buffer[9]);
            sum_gyro[1] += (int16_t)(buffer[10] << 8 | buffer[11]);
            sum_gyro[2] += (int16_t)(buffer[12] << 8 | buffer[13]);
            valid_samples++;
        }
        Delay_us(2U * 1000U);
    }

    if (valid_samples == 0U) {
        return;
    }

    gyro_bias[0] = (float32_t)sum_gyro[0] / (float32_t)valid_samples;
    gyro_bias[1] = (float32_t)sum_gyro[1] / (float32_t)valid_samples;
    gyro_bias[2] = (float32_t)sum_gyro[2] / (float32_t)valid_samples;

    /*
     * For tilt bring-up, do not zero the accelerometer here.
     * If startup happens while the board is already tilted, subtracting
     * the averaged accel sample would incorrectly cancel gravity on X/Y.
     */
    accel_bias[0] = 0.0f;
    accel_bias[1] = 0.0f;
    accel_bias[2] = 0.0f;

    is_calibrated = 1U;
}

uint8_t ICM20602_IsReady(void)
{
    return s_icm20602_ready;
}

void IMU_PROCESS(void)
{
    uint8_t buffer[ICM20602_BURST_LENGTH];

    if (!s_icm20602_ready) {
        return;
    }

    if (ICM20602_ReadRegisters(ICM20602_REG_ACCEL_XOUT_H, buffer, ICM20602_BURST_LENGTH) == HAL_OK) {
        ICM20602_ParseMotionFrame(buffer);
    }
}
