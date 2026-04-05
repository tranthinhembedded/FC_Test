/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "platform/dma.h"
#include "platform/delay.h"
#include "platform/gpio.h"
#include "platform/i2c.h"
#include "platform/spi.h"
#include "platform/tim.h"
#include "platform/usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm/pid_tuning.h"
#include "comm/telemetry.h"
#include "control/flight_control.h"
#include "input/rc_input.h"
#include "sensor/complementary_filter.h"
#include "sensor/imu_config.h"
#include "sensor/magnetometer_sensor.h"
#include "sensor/sensor_common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_LOOP_PERIOD_US            1000U
#define MAIN_LOOP_DT_RESYNC_THRESHOLD 10000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t current_time;
static uint32_t prev_time;
static uint32_t dt;
static uint32_t last_telemetry_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*
 * Legacy application entry kept for reference while we bring up
 * and validate each hardware/software block from a simpler main.c.
 */
int main_legacy(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_DMA_Init();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  Delay_Init();
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  Delay_us(200U * 1000U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  Delay_us(200U * 1000U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  FlightController_InitMotorOutputs();
  RcReceiver_Init();

  ICM20602_Init();
  Delay_us(50U * 1000U);
  HMC5883L_Init();
  Delay_us(500U * 1000U);
  ICM20602_Calibrate();

  RESET_ALL_PID();
  current_time = TIM2->CNT;
  prev_time = current_time;
  enable_motor = 0;
  ARM_Status = NOT_ARM;
  Throttle = 1000.0f;

  PidTuning_Init();
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    current_time = TIM2->CNT;
    dt = current_time - prev_time;
    prev_time = current_time;

    if (dt > MAIN_LOOP_DT_RESYNC_THRESHOLD) {
      dt = MAIN_LOOP_PERIOD_US;
    }

    PidTuning_ProcessPendingCommand();

    IMU_PROCESS();
    COMPASS_PROCESS();

    MPU6500_DATA.dt = (float32_t)dt * 1.0e-6f;
    Complimentary_Filter_Predict(&Complimentary_Filter, &MPU6500_DATA);
    if (MagCal.state == MAG_CAL_DONE) {
      Complimentary_Filter_Update(&Complimentary_Filter, &HMC5883L_DATA);
    }

    MPC();

    if (HAL_GetTick() - last_telemetry_time > 100) {
      Send_Telemetry();
      last_telemetry_time = HAL_GetTick();
    }

    while ((TIM2->CNT - current_time) < MAIN_LOOP_PERIOD_US) {
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
