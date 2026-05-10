/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : FLIGHT CONTROLLER - iNav Style, Modular Architecture
  * Updated         : Adapted from legacy monolithic main.c
  *                   - RC: iBUS via USART2 DMA (RcReceiver module)
  *                   - IMU: ICM20602 via SPI1 (imu_config module)
  *                   - MAG: HMC5883L/QMC5883L via I2C1 (magnetometer_sensor module)
  *                   - PID Tuning: USART1 DMA (pid_tuning module)
  *                   - Optical flow: USART6 direct MTF-02P parser
  *                   - Telemetry: disabled while USART1 is dedicated to optical RX
  *                   - Control: 1kHz loop, Angle / Rate mode switching
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "platform/dma.h"
#include "platform/i2c.h"
#include "platform/spi.h"
#include "platform/tim.h"
#include "platform/usart.h"
#include "platform/gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor/imu_config.h"
#include "sensor/sensor_check.h"
#include "sensor/sensor_common.h"
#include "sensor/complementary_filter.h"
#include "sensor/magnetometer_sensor.h"
#include "sensor/bmp280_sensor.h"
#include "control/flight_control.h"
#include "input/rc_input.h"
#include "comm/optical_direct.h"
#include "comm/pid_tuning.h"
#include "comm/telemetry.h"
#include "platform/system_check.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_LOOP_PERIOD_US            1000U   /* 1 kHz control loop */
#define MAIN_LOOP_OVERRUN_TOLERANCE_US 50U
#define MAIN_LOOP_DT_RESYNC_THRESHOLD  10000U  /* >10 ms → clamp dt */
#define MAIN_LOOP_DT_MAX_WINDOW_MS     1000U   /* rolling 1 s max for debugger */
#define UART1_TELEMETRY_ENABLED        1U      /* keep USART1 quiet for optical RX test */
#define TELEMETRY_PERIOD_MS             100U   /* 10 Hz telemetry */
#define MAG_UPDATE_DIV                  40U    /* 25 Hz magnetometer update */
#define BARO_UPDATE_DIV                100U    /* 10 Hz barometer update */
#define BARO_UPDATE_PHASE_DIV           20U    /* stagger baro away from mag */
#define OPTICAL_IDLE_MIN_SLACK_US      120U    /* keep this much margin before next 1kHz tick */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* --- Pre-Flight Debug / Live Expressions --- */
volatile float Debug_Roll_Deg  = 0.0f;
volatile float Debug_Pitch_Deg = 0.0f;
volatile float Debug_Yaw_Deg   = 0.0f;

volatile float Debug_PID_Roll_Out      = 0.0f;
volatile float Debug_PID_Rate_Roll_Out = 0.0f;

/**
 * Debug_Prearm_Block_Reason:
 *   0 = OK / Armed
 *   1 = Sensor error or not calibrated
 *   2 = Throttle too high to arm
 *   3 = Arm switch not activated
 *   4 = RC link lost
 *   5 = Optical flow required but not ready
 */
volatile uint8_t Debug_Prearm_Block_Reason = 1U;

/* --- Motor PWM Outputs (for Live Expressions) --- */
volatile uint16_t Motor_M2_BR_Speed = 0U; /* PWM_MOTOR[0] → TIM3 CH1 */
volatile uint16_t Motor_M1_FR_Speed = 0U; /* PWM_MOTOR[1] → TIM3 CH2 */
volatile uint16_t Motor_M4_FL_Speed = 0U; /* PWM_MOTOR[2] → TIM4 CH1 */
volatile uint16_t Motor_M3_BL_Speed = 0U; /* PWM_MOTOR[3] → TIM4 CH2 */

/* --- Telemetry --- */
#if UART1_TELEMETRY_ENABLED
uint32_t last_telemetry_time = 0U;
#endif

/* --- Loop timing diagnostics --- */
volatile uint32_t loop_dt_us       = 0U;
volatile uint32_t loop_dt_max_us   = 0U;
volatile uint32_t loop_overrun_count = 0U;
volatile uint32_t loop_exec_us     = 0U;
volatile uint32_t loop_exec_max_us = 0U;

/* --- System ready flag --- */
volatile uint8_t fc_preflight_ready = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* Start ESC idle PWM before long startup delays so the ESCs can finish
   * their power-up/arming tone sequence reliably.
   */
  FlightController_InitMotorOutputs();

  /* Startup LED blink (3×) */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  /* System health check initialization (sensor probe + calibration) */
  SystemCheck_Init();

  /* iBUS RC receiver – USART2 DMA ring buffer */
  RcReceiver_Init();

  /* µs timebase for the control loop */
  HAL_TIM_Base_Start(&htim2);

  /* Magnetometer continuous mode */
  Magnetometer_Init();

  /* Reset filter & PIDs */
  Complimentary_Filter_Reset(&Complimentary_Filter);
  RESET_ALL_PID();

  /* PID tuning UART (USART1) */
  PidTuning_Init();

  /* Direct optical flow UART (USART6 PA11/PA12) */
  OpticalDirect_Init();

  /* Safe initial state */
  enable_motor        = 0U;
  ARM_Status          = NOT_ARM;
  Throttle            = 1000.0f;
  angle_desired[0]    = 0.0f;
  angle_desired[1]    = 0.0f;
  angle_desired[2]    = 0.0f;
  angle_rate_desired[0] = 0.0f;
  angle_rate_desired[1] = 0.0f;
  angle_rate_desired[2] = 0.0f;

  MPU6500_DATA.dt        = (float32_t)MAIN_LOOP_PERIOD_US * 1.0e-6f;
  MPU6500_DATA.timestamp = TIM2->CNT;

  uint32_t last_loop_time = TIM2->CNT;
  uint32_t loop_dt_window_start_ms = HAL_GetTick();
  uint8_t  mag_div        = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms      = HAL_GetTick();
    uint32_t current_time = TIM2->CNT;
    uint32_t elapsed_since_loop = current_time - last_loop_time;

    /* USER CODE END WHILE */

    /* ================================================================
     * MAIN CONTROL LOOP – 1 kHz
     * ================================================================ */
    if (elapsed_since_loop >= MAIN_LOOP_PERIOD_US) {
      uint32_t dt_us = elapsed_since_loop;
      last_loop_time = current_time;

      if ((uint32_t)(now_ms - loop_dt_window_start_ms) >= MAIN_LOOP_DT_MAX_WINDOW_MS) {
        loop_dt_window_start_ms = now_ms;
        loop_dt_max_us = 0U;
        loop_exec_max_us = 0U;
      }

      /* Loop timing diagnostics */
      loop_dt_us = dt_us;
      if ((dt_us <= MAIN_LOOP_DT_RESYNC_THRESHOLD) && (dt_us > loop_dt_max_us)) {
        loop_dt_max_us = dt_us;
      }
      /* Clamp dt on long stalls (e.g. first iteration) */
      if (dt_us > MAIN_LOOP_DT_RESYNC_THRESHOLD) {
        dt_us = MAIN_LOOP_PERIOD_US;
      }

      MPU6500_DATA.dt        = (float32_t)dt_us * 1.0e-6f;
      MPU6500_DATA.timestamp = current_time;

      /* iBUS RC has control priority: drain UART2 DMA before MPC reads commands. */
      RcReceiver_Process_DMA_Ring_Buffer();
      RcReceiver_UpdateLinkStatus(now_ms);

      /* Keep optical UART6 parser alive even when there is little idle time. */
      OpticalDirect_Process();

      /* ---------- Sensor update ---------- */
      IMU_PROCESS();
      icm20602_last_read_ok = ICM20602_GetLastReadOk();

      Complimentary_Filter_Predict(&Complimentary_Filter, &MPU6500_DATA);

      /* Magnetometer: read at 50 Hz and correct complementary-filter yaw drift */
      mag_div++;
      if (mag_div >= MAG_UPDATE_DIV) {
        mag_div = 0U;
        COMPASS_PROCESS();
        if (Magnetometer_GetLastReadOk() != 0U) {
          Complimentary_Filter_Update(&Complimentary_Filter, &MAG_DATA_INST);
        }
      }

      /* ---------- Flight control (PID + motor mix) ---------- */
      MPC();

      /* ---------- Motor speed snapshot ---------- */
      Motor_M2_BR_Speed = (uint16_t)PWM_MOTOR[0];
      Motor_M1_FR_Speed = (uint16_t)PWM_MOTOR[1];
      Motor_M4_FL_Speed = (uint16_t)PWM_MOTOR[2];
      Motor_M3_BL_Speed = (uint16_t)PWM_MOTOR[3];

      /* ================================================================
       * BACKGROUND TASKS (Low Priority)
       * Chạy ngay sau khi tính toán 1kHz xong để tận dụng thời gian rảnh
       * ================================================================ */

      static uint8_t baro_div = BARO_UPDATE_PHASE_DIV;
      baro_div++;
      if (baro_div >= BARO_UPDATE_DIV) { /* 1000Hz / 40 = 25Hz */
          baro_div = 0;
          BARO_PROCESS();
      }

      Magnetometer_Service(now_ms);
      BMP280_Service(now_ms);

      /* System health and heartbeat update */
      SystemCheck_Update(now_ms, mag_div);

      /* PID tuning command parser (USART1) */
      PidTuning_ProcessPendingCommand();

      /* ── Measure exec time BEFORE telemetry (snprintf nặng, không tính vào loop) ── */
      {
        uint32_t exec_us = TIM2->CNT - current_time;
        loop_exec_us = exec_us;
        if ((exec_us <= MAIN_LOOP_DT_RESYNC_THRESHOLD) && (exec_us > loop_exec_max_us)) {
          loop_exec_max_us = exec_us;
        }
        if (exec_us > (MAIN_LOOP_PERIOD_US + MAIN_LOOP_OVERRUN_TOLERANCE_US)) {
          loop_overrun_count++;
        }
      }

      /* Telemetry @ 10 Hz – chạy SAU khi đo exec_us để không ảnh hưởng timing */
#if UART1_TELEMETRY_ENABLED
      if ((now_ms - last_telemetry_time) >= TELEMETRY_PERIOD_MS) {
        Send_Telemetry();
        last_telemetry_time = now_ms;
      }
#endif
    } else if ((MAIN_LOOP_PERIOD_US - elapsed_since_loop) > OPTICAL_IDLE_MIN_SLACK_US) {
      RcReceiver_Process_DMA_Ring_Buffer();
      RcReceiver_UpdateLinkStatus(HAL_GetTick());

      /* Use idle slack to catch up any remaining optical UART backlog. */
      OpticalDirect_Process();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */

/**
 * @brief  Dispatches UART idle/DMA events.
 *         - USART1 → PID tuning command parser
 *         - USART2 → handled transparently by the DMA ring-buffer;
 *           iBUS frames are parsed in RcReceiver_Process_DMA_Ring_Buffer()
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  PidTuning_HandleRxEvent(huart, Size);
}

/**
 * @brief  UART error recovery.
 *         Both pid_tuning and rc_input modules handle their own instance.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  PidTuning_HandleUartError(huart);
  OpticalDirect_HandleUartError(huart);
  RcReceiver_HandleUartError(huart);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
