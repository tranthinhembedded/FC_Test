#ifndef CORE_INC_SENSOR_GPS_KALMAN_H_
#define CORE_INC_SENSOR_GPS_KALMAN_H_

#include <stdint.h>

/**
 * @brief 1D Kalman Filter structure for Position and Velocity estimation
 */
typedef struct {
    float x; /* Position estimate (meters) */
    float v; /* Velocity estimate (m/s) */
    
    float P[2][2]; /* Error covariance matrix */
    
    float Q_accel; /* Process noise standard deviation (m/s^2) */
    float R_pos;   /* Measurement noise standard deviation (meters) */
} KalmanFilter1D_t;

/**
 * @brief Initializes the Kalman Filter with tuning parameters
 *
 * @param kf Pointer to the Kalman Filter structure
 * @param q_accel Process noise variance (Acceleration uncertainty)
 * @param r_pos Measurement noise variance (GPS Position uncertainty)
 */
void Kalman1D_Init(KalmanFilter1D_t *kf, float q_accel, float r_pos);

/**
 * @brief Prediction Step: Uses Accelerometer data to estimate next state
 *        Should be called frequently (e.g. 1000Hz like main loop)
 * 
 * @param kf Pointer to the Kalman Filter structure
 * @param accel Acceleration measured by IMU on this axis (m/s^2)
 * @param dt Time step in seconds
 */
void Kalman1D_Predict(KalmanFilter1D_t *kf, float accel, float dt);

/**
 * @brief Update Step: Uses GPS data to correct the state
 *        Should be called whenever a new GPS message arrives (e.g. 5Hz - 10Hz)
 *
 * @param kf Pointer to the Kalman Filter structure
 * @param pos_meas Position measured by GPS on this axis (meters)
 */
void Kalman1D_Update(KalmanFilter1D_t *kf, float pos_meas);

#endif /* CORE_INC_SENSOR_GPS_KALMAN_H_ */
