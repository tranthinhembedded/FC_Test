/*
 * system_check.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Antigravity
 */

#include "platform/system_check.h"
#include "main.h"
#include "sensor/sensor_check.h"
#include "sensor/imu_config.h"
#include "sensor/magnetometer_sensor.h"
#include "sensor/sensor_common.h"
#include "sensor/complementary_filter.h"
#include "sensor/bmp280_sensor.h"
#include "input/rc_input.h"
#include "control/flight_control.h"
#include "platform/i2c.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* Debug/live-expression state is defined in main.c and updated here. */
extern volatile float Debug_Roll_Deg;
extern volatile float Debug_Pitch_Deg;
extern volatile float Debug_Yaw_Deg;
extern volatile float Debug_PID_Roll_Out;
extern volatile float Debug_PID_Rate_Roll_Out;
extern volatile uint8_t Debug_Prearm_Block_Reason;
extern volatile uint8_t fc_preflight_ready;

#define SYSTEM_CHECK_SENSOR_POWER_SETTLE_MS   30U
#define SYSTEM_CHECK_I2C_RECOVER_RETRY_MS     20U
#define SYSTEM_CHECK_I2C_STARTUP_ATTEMPTS     6U
#define SYSTEM_CHECK_I2C_STARTUP_RETRY_MS     40U
#define SYSTEM_SENSOR_GOOD_STREAK_LIMIT       3U
#define SYSTEM_SENSOR_FAIL_STREAK_LIMIT       3U

static uint8_t last_arm_status = 0; /* Assuming NOT_ARM is 0 */
static uint32_t buzzer_start_time = 0;
static uint8_t buzzer_beep_count = 0;
static uint8_t buzzer_state = 0;

volatile uint8_t i2c1_recover_ok = 0U;
volatile uint8_t i2c2_recover_ok = 0U;
volatile uint8_t i2c_startup_attempts = 0U;
volatile uint8_t i2c_startup_ready = 0U;
volatile uint8_t system_sensor_instant_ok = 0U;
volatile uint8_t system_sensor_good_streak = 0U;
volatile uint8_t system_sensor_fail_streak = 0U;
volatile uint32_t system_sensor_fault_mask = 0U;
volatile uint32_t system_sensor_drop_count = 0U;

static void SystemCheck_BeepBlocking(uint8_t beeps, uint32_t on_ms, uint32_t off_ms)
{
    uint8_t i;

    for (i = 0U; i < beeps; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(on_ms);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        if ((i + 1U) < beeps) {
            HAL_Delay(off_ms);
        }
    }
}

static uint8_t SystemCheck_RecoverBus(I2C_HandleTypeDef *hi2c)
{
    uint8_t recover_ok;

    recover_ok = I2C_BusRecover(hi2c);
    if (recover_ok == 0U) {
        HAL_Delay(SYSTEM_CHECK_I2C_RECOVER_RETRY_MS);
        recover_ok = I2C_BusRecover(hi2c);
    }

    return recover_ok;
}

static uint8_t SystemCheck_BringUpI2cSensors(void)
{
    uint8_t attempt;

    i2c_startup_attempts = 0U;
    i2c_startup_ready = 0U;

    for (attempt = 0U; attempt < SYSTEM_CHECK_I2C_STARTUP_ATTEMPTS; attempt++) {
        i2c_startup_attempts = (uint8_t)(attempt + 1U);

        i2c1_recover_ok = SystemCheck_RecoverBus(&hi2c1);
        i2c2_recover_ok = SystemCheck_RecoverBus(&hi2c2);

        HAL_Delay(SYSTEM_CHECK_I2C_RECOVER_RETRY_MS);

        Magnetometer_Init();
        BMP280_Init();
        SensorCheck_RunStartupProbe();

        if ((Magnetometer_IsReady() != 0U) && (BMP280_IsReady() != 0U)) {
            i2c_startup_ready = 1U;
            return 1U;
        }

        HAL_Delay(SYSTEM_CHECK_I2C_STARTUP_RETRY_MS);
    }

    return 0U;
}

void Buzzer_TriggerBeep(uint8_t beeps) {
    buzzer_beep_count = beeps * 2; /* on/off phases */
    buzzer_start_time = HAL_GetTick();
    buzzer_state = 1;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

void Buzzer_Update(uint32_t now_ms) {
    if (buzzer_beep_count > 0) {
        if (now_ms - buzzer_start_time >= 100) { /* 100ms per beep phase */
            buzzer_start_time = now_ms;
            buzzer_state = !buzzer_state;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, buzzer_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            buzzer_beep_count--;
            if (buzzer_beep_count == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
                buzzer_state = 0;
            }
        }
    }
}

void SystemCheck_Init(void)
{
    runtime_sensors_ok = 0U;
    fc_preflight_ready = 0U;
    system_sensor_instant_ok = 0U;
    system_sensor_good_streak = 0U;
    system_sensor_fail_streak = 0U;
    system_sensor_fault_mask = 0U;
    system_sensor_drop_count = 0U;

    /* Wait for sensor power rails to stabilize after boot */
    HAL_Delay(SYSTEM_CHECK_SENSOR_POWER_SETTLE_MS);
    (void)SystemCheck_BringUpI2cSensors();

    if (BMP280_IsReady() != 0U) {
        Baro_Calibrate();
    }

    /* IMU calibration (bias removal) – only when sensor is present */
    if ((icm20602_ready != 0U) && (Magnetometer_IsReady() != 0U) && (BMP280_IsReady() != 0U)) {
        all_sensors_connected = 1U;
        /* Beep 3 times to indicate all sensors (Baro, IMU, Mag) are OK */
        SystemCheck_BeepBlocking(3U, 100U, 100U);
        
        HAL_Delay(50U);
        ICM20602_Calibrate();
    } else {
        /* Short error pattern so startup is still audible when I2C sensors fail. */
        SystemCheck_BeepBlocking(2U, 50U, 80U);
    }
}

void SystemCheck_Update(uint32_t now_ms, uint8_t mag_div)
{
    uint32_t fault_mask = 0U;

    (void)mag_div;

    /* Heartbeat LED */
    SensorCheck_UpdateHeartbeat(now_ms);

    /* Aggregate and debounce sensor health used by MPC arming logic. */
    icm20602_last_read_ok = ICM20602_GetLastReadOk();
    if ((ICM20602_IsReady() == 0U) || (icm20602_last_read_ok == 0U)) {
        fault_mask |= SYSTEM_SENSOR_FAULT_IMU;
    }
    if ((Magnetometer_IsReady() == 0U) || (Magnetometer_GetLastReadOk() == 0U)) {
        fault_mask |= SYSTEM_SENSOR_FAULT_MAG;
    }
    if ((BMP280_IsReady() == 0U) || (BMP280_GetLastReadOk() == 0U)) {
        fault_mask |= SYSTEM_SENSOR_FAULT_BARO;
    }
    if (Complimentary_Filter.Fusion_OK == 0U) {
        fault_mask |= SYSTEM_SENSOR_FAULT_FUSION;
    }
    if (is_calibrated == 0U) {
        fault_mask |= SYSTEM_SENSOR_FAULT_CALIBRATION;
    }

    system_sensor_fault_mask = fault_mask;
    system_sensor_instant_ok = (fault_mask == 0U) ? 1U : 0U;

    if (system_sensor_instant_ok != 0U) {
        system_sensor_fail_streak = 0U;
        if (system_sensor_good_streak < SYSTEM_SENSOR_GOOD_STREAK_LIMIT) {
            system_sensor_good_streak++;
        }
        if (system_sensor_good_streak >= SYSTEM_SENSOR_GOOD_STREAK_LIMIT) {
            runtime_sensors_ok = 1U;
        }
    } else {
        system_sensor_good_streak = 0U;
        if (system_sensor_fail_streak < SYSTEM_SENSOR_FAIL_STREAK_LIMIT) {
            system_sensor_fail_streak++;
        }
        if (system_sensor_fail_streak >= SYSTEM_SENSOR_FAIL_STREAK_LIMIT) {
            if (runtime_sensors_ok != 0U) {
                system_sensor_drop_count++;
            }
            runtime_sensors_ok = 0U;
        }
    }

    /* ---------- Pre-flight readiness flag ---------- */
    fc_preflight_ready = (uint8_t)(
        (runtime_sensors_ok              != 0U)
     && (rc_link_ok                      != 0U)
     && (is_calibrated                   != 0U)
     && (Complimentary_Filter.Fusion_OK  != 0U));

    /* Pre-arm block reason (for debugger) */
    if (runtime_sensors_ok == 0U) {
        Debug_Prearm_Block_Reason = 1U; /* Sensor / calibration issue */
    } else if (rc_link_ok == 0U) {
        Debug_Prearm_Block_Reason = 4U; /* RC link lost */
    } else if (RC_Raw_Throttle >= 1150U) {
        Debug_Prearm_Block_Reason = 2U; /* Lower throttle to arm */
    } else if (RC_Raw_SW_Arm <= 1500U) {
        Debug_Prearm_Block_Reason = 3U; /* Arm switch not active */
    } else if ((flight_optical_required_for_arm != 0U) && (flight_optical_arm_ok == 0U)) {
        Debug_Prearm_Block_Reason = 5U; /* Optical flow required for HOVER arm */
    } else {
        Debug_Prearm_Block_Reason = 0U; /* Ready */
    }

    Debug_Roll_Deg  = Complimentary_Filter.Euler_Angle_Deg[0];
    Debug_Pitch_Deg = Complimentary_Filter.Euler_Angle_Deg[1];
    Debug_Yaw_Deg   = Complimentary_Filter.Euler_Angle_Deg[2];

    Debug_PID_Roll_Out      = PID_ROLL.output;
    Debug_PID_Rate_Roll_Out = PID_RATE_ROLL.output;

    /* ---------- Buzzer Update & Arming Notification ---------- */
    if (ARM_Status != last_arm_status) {
        Buzzer_TriggerBeep(1);
        last_arm_status = ARM_Status;
    }
    
    Buzzer_Update(now_ms);
}
