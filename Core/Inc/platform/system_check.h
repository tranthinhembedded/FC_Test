/*
 * system_check.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Antigravity
 */

#ifndef CORE_INC_PLATFORM_SYSTEM_CHECK_H_
#define CORE_INC_PLATFORM_SYSTEM_CHECK_H_

#include <stdint.h>

#define SYSTEM_SENSOR_FAULT_IMU          (1UL << 0)
#define SYSTEM_SENSOR_FAULT_MAG          (1UL << 1)
#define SYSTEM_SENSOR_FAULT_BARO         (1UL << 2)
#define SYSTEM_SENSOR_FAULT_FUSION       (1UL << 3)
#define SYSTEM_SENSOR_FAULT_CALIBRATION  (1UL << 4)

extern volatile uint8_t fc_preflight_ready;
extern volatile uint8_t system_sensor_instant_ok;
extern volatile uint8_t system_sensor_good_streak;
extern volatile uint8_t system_sensor_fail_streak;
extern volatile uint32_t system_sensor_fault_mask;
extern volatile uint32_t system_sensor_drop_count;

/**
 * @brief Initializes the system health check module.
 *        Probes all connected sensors and sets initial pre-flight status.
 */
void SystemCheck_Init(void);

/**
 * @brief Updates the system health status, pre-flight readiness, and arming blocks.
 *        Should be called in the main loop.
 * @param now_ms Current timestamp in milliseconds.
 * @param mag_div Magnetometer divider counter from the main loop.
 */
void SystemCheck_Update(uint32_t now_ms, uint8_t mag_div);

#endif /* CORE_INC_PLATFORM_SYSTEM_CHECK_H_ */
