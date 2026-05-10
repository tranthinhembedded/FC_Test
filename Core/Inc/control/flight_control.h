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

#define FLIGHT_FAILSAFE_REASON_RC_LOSS      (1UL << 16)
#define FLIGHT_FAILSAFE_REASON_INTERLOCK    (1UL << 17)

typedef enum {
    FLIGHT_FAILSAFE_INACTIVE = 0,
    FLIGHT_FAILSAFE_LANDING,
    FLIGHT_FAILSAFE_CRITICAL,
    FLIGHT_FAILSAFE_LANDED
} FlightFailsafeState_t;

extern volatile uint32_t flight_arm_block_count;
extern volatile uint32_t flight_safety_disarm_count;
extern volatile uint32_t flight_failsafe_enter_count;
extern volatile uint32_t flight_failsafe_recover_count;
extern volatile uint32_t flight_failsafe_reason_mask;
extern volatile float32_t flight_failsafe_throttle_us;
extern volatile float32_t flight_failsafe_elapsed_s;
extern volatile uint8_t flight_failsafe_state;
extern volatile uint8_t flight_optical_arm_ok;
extern volatile uint8_t flight_optical_required_for_arm;
extern volatile uint8_t flight_optical_arm_block_reason;
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
void FlightController_SetOpticalFlowPID(const char *axis, float32_t p, float32_t i, float32_t d);

#endif /* CORE_APP_CONTROL_FLIGHTCONTROLLER_FLIGHT_CONTROLLER_H_ */
