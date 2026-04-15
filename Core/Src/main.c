/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : FLIGHT CONTROLLER - iNav Style, Modular Architecture
  * Updated         : Adapted from legacy monolithic main.c
  *                   - RC: iBUS via USART2 DMA (RcReceiver module)
  *                   - IMU: ICM20602 via SPI1 (imu_config module)
  *                   - MAG: HMC5883L via I2C1 (magnetometer_sensor module)
  *                   - PID Tuning: USART1 DMA (pid_tuning module)
  *                   - Telemetry: USART1 (telemetry module)
  *                   - Control: 1kHz loop, Angle / Rate mode switching
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor/imu_config.h"
#include "sensor/sensor_check.h"
#include "sensor/sensor_common.h"
#include "sensor/complementary_filter.h"
#include "sensor/mahony.h"
#include "sensor/magnetometer_sensor.h"
#include "control/flight_control.h"
#include "input/rc_input.h"
#include "comm/pid_tuning.h"
#include "comm/telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_LOOP_PERIOD_US            1000U   /* 1 kHz control loop */
#define MAIN_LOOP_DT_RESYNC_THRESHOLD  10000U  /* >10 ms → clamp dt */
#define TELEMETRY_PERIOD_MS            100U    /* 10 Hz telemetry */
#define MAG_UPDATE_DIV                 20U     /* 50 Hz magnetometer update */
#define MAHONY_KP                      2.5f
#define MAHONY_KI                      0.02f
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
 */
volatile uint8_t Debug_Prearm_Block_Reason = 1U;

/* --- Motor PWM Outputs (for Live Expressions) --- */
volatile uint16_t Motor_M2_BR_Speed = 0U; /* PWM_MOTOR[0] → TIM3 CH1 */
volatile uint16_t Motor_M1_FR_Speed = 0U; /* PWM_MOTOR[1] → TIM3 CH2 */
volatile uint16_t Motor_M4_FL_Speed = 0U; /* PWM_MOTOR[2] → TIM4 CH1 */
volatile uint16_t Motor_M3_BL_Speed = 0U; /* PWM_MOTOR[3] → TIM4 CH2 */

/* --- Telemetry --- */
uint32_t last_telemetry_time = 0U;

/* --- Loop timing diagnostics --- */
volatile uint32_t loop_dt_us       = 0U;
volatile uint32_t loop_dt_max_us   = 0U;
volatile uint32_t loop_overrun_count = 0U;

/* --- System ready flag --- */
volatile uint8_t fc_preflight_ready = 0U;
static MahonyAHRS_t MahonyFilter;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Startup LED blink (3×) */
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200U);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  /* Probe and record which sensors are physically present */
  HAL_Delay(50U);
  SensorCheck_RunStartupProbe();

  /* iBUS RC receiver – USART2 DMA ring buffer */
  RcReceiver_Init();

  /* Motor PWM outputs – TIM3 CH1/CH2, TIM4 CH1/CH2 @ 1000 µs idle */
  FlightController_InitMotorOutputs();

  /* µs timebase for the control loop */
  HAL_TIM_Base_Start(&htim2);

  /* Magnetometer continuous mode */
  HMC5883L_Init();

  /* IMU calibration (bias removal) – only when sensor is present */
  if (all_sensors_connected != 0U) {
    HAL_Delay(50U);
    ICM20602_Calibrate();
  }

  /* Reset filter & PIDs */
  Complimentary_Filter_Reset(&Complimentary_Filter);
  MahonyAHRS_Reset(&MahonyFilter);
  MahonyAHRS_SetGains(&MahonyFilter, MAHONY_KP, MAHONY_KI);
  RESET_ALL_PID();

  /* PID tuning UART (USART1) */
  PidTuning_Init();

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
  uint8_t  mag_div        = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms      = HAL_GetTick();
    uint32_t current_time = TIM2->CNT;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Heartbeat LED */
    SensorCheck_UpdateHeartbeat(now_ms);

    /* iBUS: parse new bytes from DMA ring buffer */
    RcReceiver_Process_DMA_Ring_Buffer();

    /* PID tuning command parser (USART1) */
    PidTuning_ProcessPendingCommand();

    /* Telemetry @ 10 Hz */
    if ((now_ms - last_telemetry_time) >= TELEMETRY_PERIOD_MS) {
      Send_Telemetry();
      last_telemetry_time = now_ms;
    }

    /* ================================================================
     * MAIN CONTROL LOOP – 1 kHz
     * ================================================================ */
    if ((current_time - last_loop_time) >= MAIN_LOOP_PERIOD_US) {
      uint32_t dt_us = current_time - last_loop_time;
      last_loop_time = current_time;

      /* Loop timing diagnostics */
      loop_dt_us = dt_us;
      if (dt_us > loop_dt_max_us) {
        loop_dt_max_us = dt_us;
      }
      if (dt_us > MAIN_LOOP_PERIOD_US) {
        loop_overrun_count++;
      }
      /* Clamp dt on long stalls (e.g. first iteration) */
      if (dt_us > MAIN_LOOP_DT_RESYNC_THRESHOLD) {
        dt_us = MAIN_LOOP_PERIOD_US;
      }

      MPU6500_DATA.dt        = (float32_t)dt_us * 1.0e-6f;
      MPU6500_DATA.timestamp = current_time;

      /* ---------- Sensor update ---------- */
      IMU_PROCESS();
      icm20602_last_read_ok = ICM20602_GetLastReadOk();

      /* Magnetometer: read at 50 Hz while the Mahony filter keeps 1 kHz IMU updates */
      mag_div++;
      if (mag_div >= MAG_UPDATE_DIV) {
        mag_div = 0U;
        COMPASS_PROCESS();
      }
      MahonyAHRS_Update(&MahonyFilter,
          &MPU6500_DATA,
          &HMC5883L_DATA,
          (uint8_t)((mag_div == 0U) && (HMC5883L_GetLastReadOk() != 0U)),
          &Complimentary_Filter);

      /* Aggregate sensor health flag used by MPC arming logic */
      runtime_sensors_ok = (uint8_t)(
          (icm20602_last_read_ok != 0U)
       && (HMC5883L_IsReady()         != 0U)
       && ((HMC5883L_GetLastReadOk()  != 0U) || (mag_div != 0U)));

      /* ---------- Flight control (PID + motor mix) ---------- */
      MPC();

      /* ---------- Pre-flight readiness flag ---------- */
      fc_preflight_ready = (uint8_t)(
          (runtime_sensors_ok              != 0U)
       && (rc_link_ok                      != 0U)
       && (is_calibrated                   != 0U)
       && (Complimentary_Filter.Fusion_OK  != 0U));

      /* ---------- Live Expressions ---------- */
      Debug_Roll_Deg  = Complimentary_Filter.Euler_Angle_Deg[0];
      Debug_Pitch_Deg = Complimentary_Filter.Euler_Angle_Deg[1];
      Debug_Yaw_Deg   = Complimentary_Filter.Euler_Angle_Deg[2];

      Debug_PID_Roll_Out      = PID_ROLL.output;
      Debug_PID_Rate_Roll_Out = PID_RATE_ROLL.output;

      /* Pre-arm block reason (for debugger) */
      if (fc_preflight_ready == 0U) {
        Debug_Prearm_Block_Reason = 1U; /* Sensor / calibration issue */
      } else if (RC_Raw_Throttle >= 1150U) {
        Debug_Prearm_Block_Reason = 2U; /* Lower throttle to arm */
      } else if (RC_Raw_SW_Arm <= 1500U) {
        Debug_Prearm_Block_Reason = 3U; /* Arm switch not active */
      } else {
        Debug_Prearm_Block_Reason = 0U; /* Ready */
      }

      /* Motor speed snapshot */
      Motor_M2_BR_Speed = (uint16_t)PWM_MOTOR[0];
      Motor_M1_FR_Speed = (uint16_t)PWM_MOTOR[1];
      Motor_M4_FL_Speed = (uint16_t)PWM_MOTOR[2];
      Motor_M3_BL_Speed = (uint16_t)PWM_MOTOR[3];
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
