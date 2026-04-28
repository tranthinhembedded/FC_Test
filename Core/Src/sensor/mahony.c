#include "sensor/mahony.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAHONY_DEFAULT_TWO_KP    (2.0f * 2.0f)
#define MAHONY_DEFAULT_TWO_KI    (2.0f * 0.0f)
#define MAHONY_INTEGRAL_LIMIT    0.30f
#define MAHONY_ACC_NORM_MIN      0.5f
#define MAHONY_ACC_NORM_MAX      2.0f
#define MAHONY_STATIONARY_GYRO_LIMIT_RAD_S  0.05f
#define MAHONY_STATIONARY_TRIM_TAU_S        1.5f

static float Mahony_Clamp(float x, float min_value, float max_value)
{
    if (x < min_value) {
        return min_value;
    }
    if (x > max_value) {
        return max_value;
    }
    return x;
}

static float Mahony_InvSqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static void Mahony_GetRawEuler(const MahonyAHRS_t *ahrs, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    float sinp;

    if ((ahrs == 0) || (roll_rad == 0) || (pitch_rad == 0) || (yaw_rad == 0)) {
        return;
    }

    *yaw_rad = atan2f(
        2.0f * (ahrs->q1 * ahrs->q2 + ahrs->q0 * ahrs->q3),
        1.0f - 2.0f * (ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3));

    sinp = 2.0f * (ahrs->q0 * ahrs->q2 - ahrs->q1 * ahrs->q3);
    sinp = Mahony_Clamp(sinp, -1.0f, 1.0f);
    *pitch_rad = asinf(sinp);

    *roll_rad = atan2f(
        2.0f * (ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3),
        1.0f - 2.0f * (ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2));
}

static void Mahony_UpdateStationaryTrim(MahonyAHRS_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt)
{
    float gyro_norm;
    float acc_norm;
    float trim_gain;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    if ((ahrs == 0) || (dt <= 0.0f)) {
        return;
    }

    gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    if ((gyro_norm > MAHONY_STATIONARY_GYRO_LIMIT_RAD_S)
        || (acc_norm < MAHONY_ACC_NORM_MIN)
        || (acc_norm > MAHONY_ACC_NORM_MAX)) {
        return;
    }

    Mahony_GetRawEuler(ahrs, &roll_rad, &pitch_rad, &yaw_rad);

    trim_gain = dt / MAHONY_STATIONARY_TRIM_TAU_S;
    trim_gain = Mahony_Clamp(trim_gain, 0.0f, 0.05f);

    /*
     * Slowly pull the displayed level reference toward zero only while the
     * board is stationary. This removes residual gyro-bias drift on roll/pitch
     * without affecting dynamic flight behaviour.
     */
    ahrs->roll_trim_rad += trim_gain * (roll_rad - ahrs->roll_trim_rad);
    ahrs->pitch_trim_rad += trim_gain * (pitch_rad - ahrs->pitch_trim_rad);
}

static void Mahony_UpdateOutput(MahonyAHRS_t *ahrs, Complimentary_Filter_t *output)
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    output->q[0] = ahrs->q0;
    output->q[1] = ahrs->q1;
    output->q[2] = ahrs->q2;
    output->q[3] = ahrs->q3;

    Mahony_GetRawEuler(ahrs, &roll_rad, &pitch_rad, &yaw_rad);

    /*
     * Export angles with aircraft sign convention:
     *   +roll  = right wing down
     *   +pitch = nose up
     * The raw Mahony quaternion in this project comes out with the opposite
     * roll/pitch sign after the sensor-axis remap, so invert only here.
     */
    output->Euler_Angle_Rad[0] = (roll_rad - ahrs->roll_trim_rad);
    output->Euler_Angle_Rad[1] = -(pitch_rad - ahrs->pitch_trim_rad);
    output->Euler_Angle_Rad[2] = yaw_rad;

    output->Euler_Angle_Deg[0] = (output->Euler_Angle_Rad[0] - output->Offset_Rad[0]) * RAD_TO_DEG;
    output->Euler_Angle_Deg[1] = (output->Euler_Angle_Rad[1] - output->Offset_Rad[1]) * RAD_TO_DEG;
    output->Euler_Angle_Deg[2] = (output->Euler_Angle_Rad[2] - output->Offset_Rad[2]) * RAD_TO_DEG;

    output->ypr[0] = output->Euler_Angle_Deg[2];
    output->ypr[1] = output->Euler_Angle_Deg[1];
    output->ypr[2] = output->Euler_Angle_Deg[0];

    output->predict_count = ahrs->predict_count;
    output->update_count = ahrs->update_count;
    output->status = Fusion_RUN;
    output->Fusion_OK = (ahrs->predict_count > 100U) ? 1U : 0U;
}

static void Mahony_InitializeFromSensors(MahonyAHRS_t *ahrs,
    float ax, float ay, float az,
    float mx, float my, float mz,
    uint8_t mag_valid,
    Complimentary_Filter_t *output)
{
    float recip_norm;
    float roll;
    float pitch;
    float yaw;
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;
    float mag_x;
    float mag_y;
    float acc_norm_sq = ax * ax + ay * ay + az * az;

    if (acc_norm_sq < 1e-12f) {
        return;
    }

    recip_norm = Mahony_InvSqrt(acc_norm_sq);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    yaw = 0.0f;

    if (mag_valid != 0U) {
        float mag_norm_sq = mx * mx + my * my + mz * mz;
        if (mag_norm_sq >= 1e-12f) {
            recip_norm = Mahony_InvSqrt(mag_norm_sq);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            cr = cosf(roll);
            sr = sinf(roll);
            cp = cosf(pitch);
            sp = sinf(pitch);

            mag_x = mx * cp + my * sr * sp + mz * cr * sp;
            mag_y = my * cr - mz * sr;
            yaw = atan2f(-mag_y, mag_x);
        }
    }

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);
    cy = cosf(yaw * 0.5f);
    sy = sinf(yaw * 0.5f);

    ahrs->q0 = cr * cp * cy + sr * sp * sy;
    ahrs->q1 = sr * cp * cy - cr * sp * sy;
    ahrs->q2 = cr * sp * cy + sr * cp * sy;
    ahrs->q3 = cr * cp * sy - sr * sp * cy;

    recip_norm = Mahony_InvSqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1
        + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
    ahrs->q0 *= recip_norm;
    ahrs->q1 *= recip_norm;
    ahrs->q2 *= recip_norm;
    ahrs->q3 *= recip_norm;
    ahrs->roll_trim_rad = roll;
    ahrs->pitch_trim_rad = pitch;
    ahrs->initialized = 1U;

    Mahony_UpdateOutput(ahrs, output);
}

static void Mahony_UpdateImuOnly(MahonyAHRS_t *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt,
    Complimentary_Filter_t *output)
{
    float recip_norm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;
    float acc_norm_sq = ax * ax + ay * ay + az * az;

    if ((dt <= 0.0f) || (dt > 0.1f)) {
        return;
    }

    if (acc_norm_sq > 1e-12f) {
        float acc_norm = sqrtf(acc_norm_sq);

        if ((acc_norm >= MAHONY_ACC_NORM_MIN) && (acc_norm <= MAHONY_ACC_NORM_MAX)) {
            recip_norm = Mahony_InvSqrt(acc_norm_sq);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            halfvx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
            halfvy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
            halfvz = ahrs->q0 * ahrs->q0 - 0.5f + ahrs->q3 * ahrs->q3;

            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            if (ahrs->twoKi > 0.0f) {
                ahrs->integralFBx += ahrs->twoKi * halfex * dt;
                ahrs->integralFBy += ahrs->twoKi * halfey * dt;
                ahrs->integralFBz += ahrs->twoKi * halfez * dt;

                ahrs->integralFBx = Mahony_Clamp(ahrs->integralFBx, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
                ahrs->integralFBy = Mahony_Clamp(ahrs->integralFBy, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
                ahrs->integralFBz = Mahony_Clamp(ahrs->integralFBz, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);

                gx += ahrs->integralFBx;
                gy += ahrs->integralFBy;
                gz += ahrs->integralFBz;
            } else {
                ahrs->integralFBx = 0.0f;
                ahrs->integralFBy = 0.0f;
                ahrs->integralFBz = 0.0f;
            }

            gx += ahrs->twoKp * halfex;
            gy += ahrs->twoKp * halfey;
            gz += ahrs->twoKp * halfez;
        }
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = ahrs->q0;
    qb = ahrs->q1;
    qc = ahrs->q2;

    ahrs->q0 += (-qb * gx - qc * gy - ahrs->q3 * gz);
    ahrs->q1 += ( qa * gx + qc * gz - ahrs->q3 * gy);
    ahrs->q2 += ( qa * gy - qb * gz + ahrs->q3 * gx);
    ahrs->q3 += ( qa * gz + qb * gy - qc * gx);

    recip_norm = Mahony_InvSqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1
        + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
    ahrs->q0 *= recip_norm;
    ahrs->q1 *= recip_norm;
    ahrs->q2 *= recip_norm;
    ahrs->q3 *= recip_norm;

    ahrs->predict_count++;
    Mahony_UpdateStationaryTrim(ahrs, gx, gy, gz, ax, ay, az, dt);
    Mahony_UpdateOutput(ahrs, output);
}

void MahonyAHRS_Reset(MahonyAHRS_t *ahrs)
{
    if (ahrs == 0) {
        return;
    }

    ahrs->twoKp = MAHONY_DEFAULT_TWO_KP;
    ahrs->twoKi = MAHONY_DEFAULT_TWO_KI;
    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;
    ahrs->roll_trim_rad = 0.0f;
    ahrs->pitch_trim_rad = 0.0f;
    ahrs->predict_count = 0U;
    ahrs->update_count = 0U;
    ahrs->initialized = 0U;
}

void MahonyAHRS_SetGains(MahonyAHRS_t *ahrs, float kp, float ki)
{
    if (ahrs == 0) {
        return;
    }

    if (kp < 0.0f) {
        kp = 0.0f;
    }
    if (ki < 0.0f) {
        ki = 0.0f;
    }

    ahrs->twoKp = 2.0f * kp;
    ahrs->twoKi = 2.0f * ki;
}

void MahonyAHRS_Update(MahonyAHRS_t *ahrs,
    const IMU_Data_t *imu,
    const MAG_DATA_t *mag,
    uint8_t mag_valid,
    Complimentary_Filter_t *output)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    float dt;
    float recip_norm;
    float q0q0;
    float q0q1;
    float q0q2;
    float q0q3;
    float q1q1;
    float q1q2;
    float q1q3;
    float q2q2;
    float q2q3;
    float q3q3;
    float hx;
    float hy;
    float bx;
    float bz;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfwx;
    float halfwy;
    float halfwz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;
    float acc_norm_sq;
    float mag_norm_sq;

    if ((ahrs == 0) || (imu == 0) || (output == 0)) {
        return;
    }

    gx = imu->w[0];
    gy = imu->w[1];
    gz = imu->w[2];

    /*
     * The project stores accelerometer as specific force with Z negative at rest.
     * Mahony expects a gravity-direction vector, so feed the negated accelerometer.
     */
    ax = -imu->acc[0];
    ay = -imu->acc[1];
    az = -imu->acc[2];
    dt = imu->dt;

    mx = 0.0f;
    my = 0.0f;
    mz = 0.0f;
    if (mag != 0) {
        mx = mag->mag_uT[0];
        my = mag->mag_uT[1];
        mz = mag->mag_uT[2];
    }

    if ((dt <= 0.0f) || (dt > 0.1f)) {
        return;
    }

    if (ahrs->initialized == 0U) {
        Mahony_InitializeFromSensors(ahrs, ax, ay, az, mx, my, mz, mag_valid, output);
        if (ahrs->initialized == 0U) {
            return;
        }
    }

    if (mag_valid == 0U) {
        Mahony_UpdateImuOnly(ahrs, gx, gy, gz, ax, ay, az, dt, output);
        return;
    }

    mag_norm_sq = mx * mx + my * my + mz * mz;
    if (mag_norm_sq < 1e-12f) {
        Mahony_UpdateImuOnly(ahrs, gx, gy, gz, ax, ay, az, dt, output);
        return;
    }

    acc_norm_sq = ax * ax + ay * ay + az * az;
    if (acc_norm_sq > 1e-12f) {
        float acc_norm = sqrtf(acc_norm_sq);

        if ((acc_norm >= MAHONY_ACC_NORM_MIN) && (acc_norm <= MAHONY_ACC_NORM_MAX)) {
            recip_norm = Mahony_InvSqrt(acc_norm_sq);
            ax *= recip_norm;
            ay *= recip_norm;
            az *= recip_norm;

            recip_norm = Mahony_InvSqrt(mag_norm_sq);
            mx *= recip_norm;
            my *= recip_norm;
            mz *= recip_norm;

            q0q0 = ahrs->q0 * ahrs->q0;
            q0q1 = ahrs->q0 * ahrs->q1;
            q0q2 = ahrs->q0 * ahrs->q2;
            q0q3 = ahrs->q0 * ahrs->q3;
            q1q1 = ahrs->q1 * ahrs->q1;
            q1q2 = ahrs->q1 * ahrs->q2;
            q1q3 = ahrs->q1 * ahrs->q3;
            q2q2 = ahrs->q2 * ahrs->q2;
            q2q3 = ahrs->q2 * ahrs->q3;
            q3q3 = ahrs->q3 * ahrs->q3;

            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3)
                + my * (q1q2 - q0q3)
                + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3)
                + my * (0.5f - q1q1 - q3q3)
                + mz * (q2q3 - q0q1));
            bx = sqrtf(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2)
                + my * (q2q3 + q0q1)
                + mz * (0.5f - q1q1 - q2q2));

            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;

            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            if (ahrs->twoKi > 0.0f) {
                ahrs->integralFBx += ahrs->twoKi * halfex * dt;
                ahrs->integralFBy += ahrs->twoKi * halfey * dt;
                ahrs->integralFBz += ahrs->twoKi * halfez * dt;

                ahrs->integralFBx = Mahony_Clamp(ahrs->integralFBx, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
                ahrs->integralFBy = Mahony_Clamp(ahrs->integralFBy, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);
                ahrs->integralFBz = Mahony_Clamp(ahrs->integralFBz, -MAHONY_INTEGRAL_LIMIT, MAHONY_INTEGRAL_LIMIT);

                gx += ahrs->integralFBx;
                gy += ahrs->integralFBy;
                gz += ahrs->integralFBz;
            } else {
                ahrs->integralFBx = 0.0f;
                ahrs->integralFBy = 0.0f;
                ahrs->integralFBz = 0.0f;
            }

            gx += ahrs->twoKp * halfex;
            gy += ahrs->twoKp * halfey;
            gz += ahrs->twoKp * halfez;
        }
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = ahrs->q0;
    qb = ahrs->q1;
    qc = ahrs->q2;

    ahrs->q0 += (-qb * gx - qc * gy - ahrs->q3 * gz);
    ahrs->q1 += ( qa * gx + qc * gz - ahrs->q3 * gy);
    ahrs->q2 += ( qa * gy - qb * gz + ahrs->q3 * gx);
    ahrs->q3 += ( qa * gz + qb * gy - qc * gx);

    recip_norm = Mahony_InvSqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1
        + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
    ahrs->q0 *= recip_norm;
    ahrs->q1 *= recip_norm;
    ahrs->q2 *= recip_norm;
    ahrs->q3 *= recip_norm;

    ahrs->predict_count++;
    ahrs->update_count++;
    Mahony_UpdateStationaryTrim(ahrs, gx, gy, gz, ax, ay, az, dt);
    Mahony_UpdateOutput(ahrs, output);
}
