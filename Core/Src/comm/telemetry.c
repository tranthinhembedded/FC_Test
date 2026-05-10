#include "comm/telemetry.h"

#include "platform/usart.h"
#include "sensor/sensor_common.h"
#include "control/flight_control.h"
#include "input/rc_input.h"
#include "platform/system_check.h"

#include <stdio.h>
#include <stdlib.h>

#define F_SGN(x) (((x) < 0.0f) ? "-" : "")
#define F_INT(x) (abs((int)(x)))
#define F_DC1(x) (abs((int)((x) * 10.0f)) % 10)
#define F_DC2(x) (abs((int)((x) * 100.0f)) % 100)

/* Báo cho compiler biết biến loop_exec_us đang nằm ở file main.c */
extern volatile uint32_t loop_exec_us;

void Send_Telemetry(void)
{
    static char tx_buf[512]; // Tăng kích thước buffer cho mag data
    int len;
    FlightController_OpticalFlowState_t opt_state;

    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    FlightController_GetOpticalFlowState(&opt_state);

    int armStatus = (ARM_Status == ARM) ? 1 : 0;
    int flightMode = (RC_Raw_SW_PosHold > 1500U) ? 2 : ((MPC_Status == HOVER) ? 1 : 0);
    int hz = (int)(1.0f / MPU6500_DATA.dt);
    int exec_time = (int)loop_exec_us;
    float vbat = 11.1f; // TODO: Connect to real ADC battery reading

    float a0 = Complimentary_Filter.Euler_Angle_Deg[0];
    float a1 = Complimentary_Filter.Euler_Angle_Deg[1];
    float a2 = Complimentary_Filter.Euler_Angle_Deg[2];
    
    float at0 = angle_desired[0];
    float at1 = angle_desired[1];
    float at2 = angle_desired[2];
    
    float r0 = MPU6500_DATA.w[0] * 57.2957795f;
    float r1 = MPU6500_DATA.w[1] * 57.2957795f;
    float r2 = MPU6500_DATA.w[2] * 57.2957795f;
    
    float rt0 = angle_rate_desired[0];
    float rt1 = angle_rate_desired[1];
    float rt2 = angle_rate_desired[2];
    
    float f0 = opt_state.velocity_body_mps[0];
    float f1 = opt_state.velocity_body_mps[1];

    len = snprintf(tx_buf, sizeof(tx_buf),
        "{\"ang\":[%s%d.%d,%s%d.%d,%s%d.%d],\"angT\":[%s%d.%d,%s%d.%d,%s%d.%d],\"rate\":[%s%d.%d,%s%d.%d,%s%d.%d],\"rateT\":[%s%d.%d,%s%d.%d,%s%d.%d],\"flow\":[%s%d.%02d,%s%d.%02d],\"rc\":[%u,%u,%u,%u],\"mot\":[%u,%u,%u,%u],\"sys\":[%d,%d,%d,%d,%d,%d,%d,%d,%d],\"mag\":[%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d,%s%d.%02d],\"v\":%s%d.%d,\"hz\":%d,\"exec\":%d}\n",
        F_SGN(a0), F_INT(a0), F_DC1(a0),
        F_SGN(a1), F_INT(a1), F_DC1(a1),
        F_SGN(a2), F_INT(a2), F_DC1(a2),
        F_SGN(at0), F_INT(at0), F_DC1(at0),
        F_SGN(at1), F_INT(at1), F_DC1(at1),
        F_SGN(at2), F_INT(at2), F_DC1(at2),
        F_SGN(r0), F_INT(r0), F_DC1(r0),
        F_SGN(r1), F_INT(r1), F_DC1(r1),
        F_SGN(r2), F_INT(r2), F_DC1(r2),
        F_SGN(rt0), F_INT(rt0), F_DC1(rt0),
        F_SGN(rt1), F_INT(rt1), F_DC1(rt1),
        F_SGN(rt2), F_INT(rt2), F_DC1(rt2),
        F_SGN(f0), F_INT(f0), F_DC2(f0),
        F_SGN(f1), F_INT(f1), F_DC2(f1),
        (unsigned int)RC_Raw_Roll, (unsigned int)RC_Raw_Pitch, (unsigned int)RC_Raw_Yaw, (unsigned int)RC_Raw_Throttle,
        (unsigned int)PWM_TIMER[0], (unsigned int)PWM_TIMER[1], (unsigned int)PWM_TIMER[2], (unsigned int)PWM_TIMER[3],
        armStatus, (int)flight_optical_arm_block_reason, flightMode, (int)rc_link_ok, (int)flight_failsafe_state, (int)system_sensor_fault_mask, (int)opt_state.healthy,
        (int)MagCal.state, (int)MagCal.samples,
        F_SGN(MagCal.offset[0]), F_INT(MagCal.offset[0]), F_DC2(MagCal.offset[0]),
        F_SGN(MagCal.offset[1]), F_INT(MagCal.offset[1]), F_DC2(MagCal.offset[1]),
        F_SGN(MagCal.offset[2]), F_INT(MagCal.offset[2]), F_DC2(MagCal.offset[2]),
        F_SGN(MagCal.scale[0]), F_INT(MagCal.scale[0]), F_DC2(MagCal.scale[0]),
        F_SGN(MagCal.scale[1]), F_INT(MagCal.scale[1]), F_DC2(MagCal.scale[1]),
        F_SGN(MagCal.scale[2]), F_INT(MagCal.scale[2]), F_DC2(MagCal.scale[2]),
        F_SGN(vbat), F_INT(vbat), F_DC1(vbat),
        hz,
        exec_time);

    if (len > 0) {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, (uint16_t)len);
    }
}
