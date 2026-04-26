/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
extern volatile uint32_t i2c1_recovery_count;
extern volatile uint32_t i2c2_recovery_count;
extern volatile uint32_t i2c1_recovery_fail_count;
extern volatile uint32_t i2c2_recovery_fail_count;
extern volatile uint32_t i2c1_last_error_code;
extern volatile uint32_t i2c2_last_error_code;
extern volatile uint8_t i2c1_scl_high;
extern volatile uint8_t i2c1_sda_high;
extern volatile uint8_t i2c2_scl_high;
extern volatile uint8_t i2c2_sda_high;

uint8_t I2C_BusRecover(I2C_HandleTypeDef *hi2c);
uint8_t I2C_IsDeviceReadyRecover(I2C_HandleTypeDef *hi2c,
                                 uint16_t DevAddress,
                                 uint32_t Trials,
                                 uint32_t Timeout);
uint8_t I2C_Mem_ReadRecover(I2C_HandleTypeDef *hi2c,
                            uint16_t DevAddress,
                            uint16_t MemAddress,
                            uint16_t MemAddSize,
                            uint8_t *pData,
                            uint16_t Size,
                            uint32_t Timeout);
uint8_t I2C_Mem_WriteRecover(I2C_HandleTypeDef *hi2c,
                             uint16_t DevAddress,
                             uint16_t MemAddress,
                             uint16_t MemAddSize,
                             uint8_t *pData,
                             uint16_t Size,
                             uint32_t Timeout);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

