#ifndef CORE_APP_CONTROL_FLIGHTCONTROLLER_FLIGHT_CONTROLLER_H_
#define CORE_APP_CONTROL_FLIGHTCONTROLLER_FLIGHT_CONTROLLER_H_

#include "arm_math.h"

#include <stdint.h>

typedef struct {
    float32_t position_earth_m[2];
    float32_t hold_position_earth_m[2];
    float32_t velocity_body_mps[2];
    float32_t velocity_earth_mps[2];
    float32_t correction_deg[2];
    float32_t altitude_m;
    float32_t quality;
    float32_t data_age_s;
    uint8_t healthy;
    uint8_t hold_locked;
} FlightController_OpticalFlowState_t;

void FlightController_InitMotorOutputs(void);
void RESET_ALL_PID(void);
void MPC(void);
void FlightController_ResetOpticalFlowHold(void);
void FlightController_UpdateOpticalFlowVelocity(float32_t velocity_x_body_mps,
                                                float32_t velocity_y_body_mps,
                                                float32_t altitude_m,
                                                float32_t quality,
                                                float32_t dt);
void FlightController_GetOpticalFlowState(FlightController_OpticalFlowState_t *state_out);

#endif /* CORE_APP_CONTROL_FLIGHTCONTROLLER_FLIGHT_CONTROLLER_H_ */
