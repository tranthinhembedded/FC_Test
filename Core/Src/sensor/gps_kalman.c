#include "sensor/gps_kalman.h"

void Kalman1D_Init(KalmanFilter1D_t *kf, float q_accel, float r_pos)
{
    kf->x = 0.0f;
    kf->v = 0.0f;
    
    /* Initialize covariance matrix with some initial uncertainty */
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    
    kf->Q_accel = q_accel;
    kf->R_pos = r_pos;
}

void Kalman1D_Predict(KalmanFilter1D_t *kf, float accel, float dt)
{
    /* 
     * 1. State Prediction:
     * x(k) = x(k-1) + v(k-1)*dt + 0.5*a*dt^2
     * v(k) = v(k-1) + a*dt
     */
    kf->x = kf->x + kf->v * dt + 0.5f * accel * dt * dt;
    kf->v = kf->v + accel * dt;
    
    /*
     * 2. Covariance Prediction:
     * P(k) = A * P(k-1) * A^T + Q
     * A = [1, dt]
     *     [0,  1]
     */
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];
    
    kf->P[0][0] = P00 + dt * (P10 + P01) + dt2 * P11 + 0.25f * dt4 * kf->Q_accel;
    kf->P[0][1] = P01 + dt * P11 + 0.5f * dt3 * kf->Q_accel;
    kf->P[1][0] = P10 + dt * P11 + 0.5f * dt3 * kf->Q_accel;
    kf->P[1][1] = P11 + dt2 * kf->Q_accel;
}

void Kalman1D_Update(KalmanFilter1D_t *kf, float pos_meas)
{
    /* 
     * 1. Calculate Innovation (Error):
     * y = z - H * x
     * (We only measure Position, so H = [1, 0])
     */
    float y = pos_meas - kf->x;
    
    /* 
     * 2. Calculate Innovation Covariance:
     * S = H * P * H^T + R
     */
    float S = kf->P[0][0] + kf->R_pos;
    
    /* 
     * 3. Calculate Kalman Gain:
     * K = P * H^T * S^-1
     */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    
    /* 
     * 4. State Update:
     * x(k) = x(k) + K * y
     */
    kf->x = kf->x + K0 * y;
    kf->v = kf->v + K1 * y;
    
    /* 
     * 5. Covariance Update:
     * P(k) = (I - K * H) * P
     */
    float P00_new = (1.0f - K0) * kf->P[0][0];
    float P01_new = (1.0f - K0) * kf->P[0][1];
    float P10_new = -K1 * kf->P[0][0] + kf->P[1][0];
    float P11_new = -K1 * kf->P[0][1] + kf->P[1][1];
    
    kf->P[0][0] = P00_new;
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P10_new;
    kf->P[1][1] = P11_new;
}
