/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "platform/i2c.h"

/* USER CODE BEGIN 0 */
#define I2C_BUS_CLOCK_HZ              400000U
#define I2C_UNSTICK_CLOCK_US          10U
#define I2C_UNSTICK_STRETCH_TIMEOUT_US 500U
#define I2C_UNSTICK_PULSE_COUNT       9U

volatile uint32_t i2c1_recovery_count = 0U;
volatile uint32_t i2c2_recovery_count = 0U;
volatile uint32_t i2c1_recovery_fail_count = 0U;
volatile uint32_t i2c2_recovery_fail_count = 0U;
volatile uint32_t i2c1_last_error_code = HAL_I2C_ERROR_NONE;
volatile uint32_t i2c2_last_error_code = HAL_I2C_ERROR_NONE;
volatile uint8_t i2c1_scl_high = 0U;
volatile uint8_t i2c1_sda_high = 0U;
volatile uint8_t i2c2_scl_high = 0U;
volatile uint8_t i2c2_sda_high = 0U;

static uint8_t s_i2c_delay_ready = 0U;

static uint8_t I2C_IsSupportedBus(const I2C_HandleTypeDef *hi2c)
{
  if (hi2c == 0) {
    return 0U;
  }

  return (uint8_t)((hi2c->Instance == I2C1) || (hi2c->Instance == I2C2));
}

static volatile uint32_t *I2C_GetRecoveryCounter(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c != 0) && (hi2c->Instance == I2C1)) {
    return &i2c1_recovery_count;
  }
  if ((hi2c != 0) && (hi2c->Instance == I2C2)) {
    return &i2c2_recovery_count;
  }
  return 0;
}

static volatile uint32_t *I2C_GetRecoveryFailCounter(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c != 0) && (hi2c->Instance == I2C1)) {
    return &i2c1_recovery_fail_count;
  }
  if ((hi2c != 0) && (hi2c->Instance == I2C2)) {
    return &i2c2_recovery_fail_count;
  }
  return 0;
}

static volatile uint32_t *I2C_GetLastErrorSlot(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c != 0) && (hi2c->Instance == I2C1)) {
    return &i2c1_last_error_code;
  }
  if ((hi2c != 0) && (hi2c->Instance == I2C2)) {
    return &i2c2_last_error_code;
  }
  return 0;
}

static void I2C_RecordError(I2C_HandleTypeDef *hi2c, uint32_t error_code)
{
  volatile uint32_t *slot = I2C_GetLastErrorSlot(hi2c);

  if (slot != 0) {
    *slot = error_code;
  }
}

static void I2C_RecordLineState(I2C_HandleTypeDef *hi2c, GPIO_PinState scl, GPIO_PinState sda)
{
  if ((hi2c != 0) && (hi2c->Instance == I2C1)) {
    i2c1_scl_high = (scl == GPIO_PIN_SET) ? 1U : 0U;
    i2c1_sda_high = (sda == GPIO_PIN_SET) ? 1U : 0U;
  } else if ((hi2c != 0) && (hi2c->Instance == I2C2)) {
    i2c2_scl_high = (scl == GPIO_PIN_SET) ? 1U : 0U;
    i2c2_sda_high = (sda == GPIO_PIN_SET) ? 1U : 0U;
  }
}

static void I2C_DelayInit(void)
{
  if (s_i2c_delay_ready != 0U) {
    return;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_i2c_delay_ready = 1U;
}

static void I2C_DelayUs(uint32_t us)
{
  uint32_t ticks;
  uint32_t start;

  I2C_DelayInit();
  ticks = (SystemCoreClock / 1000000U) * us;
  start = DWT->CYCCNT;

  while ((DWT->CYCCNT - start) < ticks) {
  }
}

static void I2C_SetPinsGpioOpenDrain(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  if (hi2c->Instance == I2C1) {
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  } else if (hi2c->Instance == I2C2) {
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_3, GPIO_PIN_SET);
  }
}

static void I2C_SetPinsAfOpenDrain(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  if (hi2c->Instance == I2C1) {
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  } else if (hi2c->Instance == I2C2) {
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

static GPIO_PinState I2C_ReadScl(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
  }
  if (hi2c->Instance == I2C2) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
  }
  return GPIO_PIN_RESET;
}

static GPIO_PinState I2C_ReadSda(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
  }
  if (hi2c->Instance == I2C2) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
  }
  return GPIO_PIN_RESET;
}

static void I2C_WriteScl(I2C_HandleTypeDef *hi2c, GPIO_PinState state)
{
  if (hi2c->Instance == I2C1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, state);
  } else if (hi2c->Instance == I2C2) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, state);
  }
}

static void I2C_WriteSda(I2C_HandleTypeDef *hi2c, GPIO_PinState state)
{
  if (hi2c->Instance == I2C1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, state);
  } else if (hi2c->Instance == I2C2) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, state);
  }
}

static uint8_t I2C_WaitSclHigh(I2C_HandleTypeDef *hi2c, uint32_t timeout_us)
{
  uint32_t waited_us = 0U;

  while (I2C_ReadScl(hi2c) == GPIO_PIN_RESET) {
    if (waited_us >= timeout_us) {
      return 0U;
    }
    I2C_DelayUs(I2C_UNSTICK_CLOCK_US);
    waited_us += I2C_UNSTICK_CLOCK_US;
  }

  return 1U;
}

static uint8_t I2C_PulseClock(I2C_HandleTypeDef *hi2c)
{
  if (I2C_WaitSclHigh(hi2c, I2C_UNSTICK_STRETCH_TIMEOUT_US) == 0U) {
    return 0U;
  }

  I2C_WriteScl(hi2c, GPIO_PIN_RESET);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
  I2C_WriteScl(hi2c, GPIO_PIN_SET);
  if (I2C_WaitSclHigh(hi2c, I2C_UNSTICK_STRETCH_TIMEOUT_US) == 0U) {
    return 0U;
  }
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
  return 1U;
}

static void I2C_GenerateStop(I2C_HandleTypeDef *hi2c)
{
  I2C_WriteScl(hi2c, GPIO_PIN_RESET);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
  I2C_WriteSda(hi2c, GPIO_PIN_RESET);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
  I2C_WriteScl(hi2c, GPIO_PIN_SET);
  (void)I2C_WaitSclHigh(hi2c, I2C_UNSTICK_STRETCH_TIMEOUT_US);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
  I2C_WriteSda(hi2c, GPIO_PIN_SET);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US / 2U);
}

static void I2C_PeripheralReset(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_FORCE_RESET();
    I2C_DelayUs(100U);
    __HAL_RCC_I2C1_RELEASE_RESET();
    I2C_DelayUs(100U);
  } else if (hi2c->Instance == I2C2) {
    __HAL_RCC_I2C2_FORCE_RESET();
    I2C_DelayUs(100U);
    __HAL_RCC_I2C2_RELEASE_RESET();
    I2C_DelayUs(100U);
  }
}

static void I2C_ReInitPeripheral(I2C_HandleTypeDef *hi2c)
{
  I2C_SetPinsAfOpenDrain(hi2c);

  if (hi2c->Instance == I2C1) {
    MX_I2C1_Init();
  } else if (hi2c->Instance == I2C2) {
    MX_I2C2_Init();
  }
}

static uint8_t I2C_PrepareForTransfer(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  if ((HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
      || (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET)) {
    if (timeout_ms <= 1U) {
      I2C_RecordError(hi2c, HAL_I2C_ERROR_TIMEOUT);
      return 0U;
    }
    (void)I2C_BusRecover(hi2c);
  }

  return (uint8_t)((HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY)
                && (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == RESET));
}

static uint8_t I2C_ShouldRecoverAfterFailure(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status)
{
  uint32_t error_code;

  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) != RESET)
      || (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)) {
    return 1U;
  }

  if (status == HAL_TIMEOUT) {
    return 1U;
  }

  error_code = HAL_I2C_GetError(hi2c);
  if ((error_code & (HAL_I2C_ERROR_BERR
                   | HAL_I2C_ERROR_ARLO
                   | HAL_I2C_ERROR_OVR
                   | HAL_I2C_ERROR_DMA
                   | HAL_I2C_ERROR_TIMEOUT
                   | HAL_I2C_ERROR_SIZE)) != 0U) {
    return 1U;
  }

  return 0U;
}

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_BUS_CLOCK_HZ;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = I2C_BUS_CLOCK_HZ;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB3     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB3     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t I2C_BusRecover(I2C_HandleTypeDef *hi2c)
{
  GPIO_PinState scl;
  GPIO_PinState sda;
  volatile uint32_t *recover_counter;
  volatile uint32_t *recover_fail_counter;
  uint8_t pulse_idx;

  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  recover_counter = I2C_GetRecoveryCounter(hi2c);
  recover_fail_counter = I2C_GetRecoveryFailCounter(hi2c);

  HAL_I2C_DeInit(hi2c);
  I2C_SetPinsGpioOpenDrain(hi2c);
  I2C_WriteScl(hi2c, GPIO_PIN_SET);
  I2C_WriteSda(hi2c, GPIO_PIN_SET);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US);

  scl = I2C_ReadScl(hi2c);
  sda = I2C_ReadSda(hi2c);
  I2C_RecordLineState(hi2c, scl, sda);

  if ((scl == GPIO_PIN_RESET)
      && (I2C_WaitSclHigh(hi2c, I2C_UNSTICK_STRETCH_TIMEOUT_US) == 0U)) {
    if (recover_fail_counter != 0) {
      (*recover_fail_counter)++;
    }
    I2C_PeripheralReset(hi2c);
    I2C_ReInitPeripheral(hi2c);
    I2C_RecordError(hi2c, HAL_I2C_ERROR_TIMEOUT);
    return 0U;
  }

  for (pulse_idx = 0U; pulse_idx < I2C_UNSTICK_PULSE_COUNT; pulse_idx++) {
    if (I2C_PulseClock(hi2c) == 0U) {
      break;
    }
  }

  I2C_GenerateStop(hi2c);
  I2C_PeripheralReset(hi2c);
  I2C_ReInitPeripheral(hi2c);

  I2C_SetPinsGpioOpenDrain(hi2c);
  I2C_DelayUs(I2C_UNSTICK_CLOCK_US);
  scl = I2C_ReadScl(hi2c);
  sda = I2C_ReadSda(hi2c);
  I2C_RecordLineState(hi2c, scl, sda);
  I2C_SetPinsAfOpenDrain(hi2c);

  if ((scl == GPIO_PIN_SET) && (sda == GPIO_PIN_SET)) {
    if (recover_counter != 0) {
      (*recover_counter)++;
    }
    I2C_RecordError(hi2c, HAL_I2C_ERROR_NONE);
    return 1U;
  }

  if (recover_fail_counter != 0) {
    (*recover_fail_counter)++;
  }
  I2C_RecordError(hi2c, HAL_I2C_ERROR_TIMEOUT);
  return 0U;
}

uint8_t I2C_IsDeviceReadyRecover(I2C_HandleTypeDef *hi2c,
                                 uint16_t DevAddress,
                                 uint32_t Trials,
                                 uint32_t Timeout)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t attempt;

  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  if (I2C_PrepareForTransfer(hi2c, Timeout) == 0U) {
    return 0U;
  }

  for (attempt = 0U; attempt < 2U; attempt++) {
    status = HAL_I2C_IsDeviceReady(hi2c, DevAddress, Trials, Timeout);
    if (status == HAL_OK) {
      I2C_RecordError(hi2c, HAL_I2C_ERROR_NONE);
      return 1U;
    }

    I2C_RecordError(hi2c, HAL_I2C_GetError(hi2c));
    if (((attempt + 1U) < 2U) && (I2C_ShouldRecoverAfterFailure(hi2c, status) != 0U)) {
      (void)I2C_BusRecover(hi2c);
    }
  }

  return 0U;
}

uint8_t I2C_Mem_ReadRecover(I2C_HandleTypeDef *hi2c,
                            uint16_t DevAddress,
                            uint16_t MemAddress,
                            uint16_t MemAddSize,
                            uint8_t *pData,
                            uint16_t Size,
                            uint32_t Timeout)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t attempt;
  uint32_t max_attempts;

  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  if (I2C_PrepareForTransfer(hi2c, Timeout) == 0U) {
    return 0U;
  }

  max_attempts = (Timeout <= 1U) ? 1U : 2U;
  for (attempt = 0U; attempt < max_attempts; attempt++) {
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
    if (status == HAL_OK) {
      I2C_RecordError(hi2c, HAL_I2C_ERROR_NONE);
      return 1U;
    }

    I2C_RecordError(hi2c, HAL_I2C_GetError(hi2c));
    if (((attempt + 1U) < max_attempts) && (I2C_ShouldRecoverAfterFailure(hi2c, status) != 0U)) {
      (void)I2C_BusRecover(hi2c);
    }
  }

  return 0U;
}

uint8_t I2C_Mem_WriteRecover(I2C_HandleTypeDef *hi2c,
                             uint16_t DevAddress,
                             uint16_t MemAddress,
                             uint16_t MemAddSize,
                             uint8_t *pData,
                             uint16_t Size,
                             uint32_t Timeout)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t attempt;
  uint32_t max_attempts;

  if (!I2C_IsSupportedBus(hi2c)) {
    return 0U;
  }

  if (I2C_PrepareForTransfer(hi2c, Timeout) == 0U) {
    return 0U;
  }

  max_attempts = (Timeout <= 1U) ? 1U : 2U;
  for (attempt = 0U; attempt < max_attempts; attempt++) {
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
    if (status == HAL_OK) {
      I2C_RecordError(hi2c, HAL_I2C_ERROR_NONE);
      return 1U;
    }

    I2C_RecordError(hi2c, HAL_I2C_GetError(hi2c));
    if (((attempt + 1U) < max_attempts) && (I2C_ShouldRecoverAfterFailure(hi2c, status) != 0U)) {
      (void)I2C_BusRecover(hi2c);
    }
  }

  return 0U;
}

/* USER CODE END 1 */
