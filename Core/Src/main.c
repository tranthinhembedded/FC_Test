/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Component test harness entry point.
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
#include "sensor/complementary_filter.h"
#include "sensor/imu_config.h"
#include "sensor/magnetometer_sensor.h"
#include "sensor/sensor_common.h"

#include <stdarg.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    APP_TEST_LED = 0,
    APP_TEST_UART1,
    APP_TEST_IMU,
    APP_TEST_MAG
} AppTestMode_t;

/* Private define ------------------------------------------------------------*/
#define ACTIVE_TEST                APP_TEST_IMU
#define HEARTBEAT_MS               250U
#define SENSOR_STREAM_MS           100U
#define APP_TEST_IMU_PERIOD_US     1000U
#define APP_TEST_IMU_DT_MAX_US     20000U
#define LOG_BUFFER_SIZE            160U

/* Private variables ---------------------------------------------------------*/
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_last_stream_ms = 0U;
static uint32_t s_prev_imu_time_us = 0U;
static uint32_t s_zero_capture_count = 0U;

/*
 * Live Watch variables:
 * - dt: actual loop period in seconds
 * - loop_hz: actual loop frequency in Hz
 * - compute_dt_us: processing time before the pacing wait
 * - max_dt: maximum observed processing time in microseconds
 */
volatile float dt = 0.0f;
volatile float loop_hz = 0.0f;
volatile uint32_t compute_dt_us = 0U;
volatile uint32_t max_dt = 0U;
volatile float loop_dt_s = 0.0f;
volatile uint32_t max_compute_dt_us = 0U;
volatile float roll_deg = 0.0f;
volatile float pitch_deg = 0.0f;
volatile float yaw_deg = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static const char *App_GetTestName(AppTestMode_t mode);
static void App_InitCommon(void);
static void App_InitSelectedTest(void);
static void App_RunSelectedTest(void);
static void App_RunLedTest(void);
static void App_RunUart1Test(void);
static void App_RunImuTest(void);
static void App_RunMagTest(void);
static void App_SendLog(const char *fmt, ...);
static void App_ToggleHeartbeat(uint32_t now_ms);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    App_InitCommon();
    App_InitSelectedTest();

    while (1) {
        App_RunSelectedTest();
    }
}

static const char *App_GetTestName(AppTestMode_t mode)
{
    switch (mode) {
    case APP_TEST_LED:
        return "LED";
    case APP_TEST_UART1:
        return "UART1";
    case APP_TEST_IMU:
        return "IMU";
    case APP_TEST_MAG:
        return "MAG";
    default:
        return "UNKNOWN";
    }
}

static void App_InitCommon(void)
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    Delay_Init();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

static void App_InitSelectedTest(void)
{
    switch (ACTIVE_TEST) {
    case APP_TEST_LED:
        break;

    case APP_TEST_UART1:
        MX_USART1_UART_Init();
        App_SendLog("Boot test mode: %s\r\n", App_GetTestName(ACTIVE_TEST));
        break;

    case APP_TEST_IMU:
        MX_SPI1_Init();
        MX_I2C1_Init();
        MX_USART1_UART_Init();

        ICM20602_Init();
        Delay_ms_blocking(50U);
        HMC5883L_Init();
        Delay_ms_blocking(50U);
        ICM20602_Calibrate();
        Complimentary_Filter_Reset(&Complimentary_Filter);
        s_prev_imu_time_us = TIM2->CNT;
        s_zero_capture_count = 0U;
        roll_deg = 0.0f;
        pitch_deg = 0.0f;
        yaw_deg = 0.0f;

        App_SendLog(
            "Boot test mode: %s, ICM-20602: %s\r\n",
            App_GetTestName(ACTIVE_TEST),
            ICM20602_IsReady() ? "ready" : "not detected"
        );
        break;

    case APP_TEST_MAG:
        MX_I2C1_Init();
        MX_USART1_UART_Init();

        HMC5883L_Init();

        App_SendLog("Boot test mode: %s\r\n", App_GetTestName(ACTIVE_TEST));
        break;

    default:
        Error_Handler();
        break;
    }
}

static void App_RunSelectedTest(void)
{
    switch (ACTIVE_TEST) {
    case APP_TEST_LED:
        App_RunLedTest();
        break;

    case APP_TEST_UART1:
        App_RunUart1Test();
        break;

    case APP_TEST_IMU:
        App_RunImuTest();
        break;

    case APP_TEST_MAG:
        App_RunMagTest();
        break;

    default:
        Error_Handler();
        break;
    }
}

static void App_RunLedTest(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    Delay_ms_blocking(HEARTBEAT_MS);
}

static void App_RunUart1Test(void)
{
    uint32_t now_ms = HAL_GetTick();

    App_ToggleHeartbeat(now_ms);

    if ((now_ms - s_last_stream_ms) >= SENSOR_STREAM_MS) {
        s_last_stream_ms = now_ms;
        App_SendLog("UART1 heartbeat %lu ms\r\n", now_ms);
    }
}

static void App_RunImuTest(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t loop_start_us = TIM2->CNT;
    uint32_t current_time_us = loop_start_us;
    uint32_t dt_us = current_time_us - s_prev_imu_time_us;
    uint32_t compute_time_us;

    s_prev_imu_time_us = current_time_us;
    if (dt_us == 0U || dt_us > APP_TEST_IMU_DT_MAX_US) {
        dt_us = APP_TEST_IMU_PERIOD_US;
    }

    IMU_PROCESS();
    dt = (float32_t)dt_us * 1.0e-6f;
    loop_dt_s = dt;
    loop_hz = 1.0f / dt;
    COMPASS_PROCESS();
    MPU6500_DATA.dt = dt;
    Complimentary_Filter_Predict(&Complimentary_Filter, &MPU6500_DATA);
    Complimentary_Filter_Update(&Complimentary_Filter, &HMC5883L_DATA);

    if (Complimentary_Filter.status == Fusion_RUN && s_zero_capture_count < 500U) {
        s_zero_capture_count++;

        if (s_zero_capture_count == 500U) {
            Complimentary_Filter.Offset_Rad[0] = Complimentary_Filter.Euler_Angle_Rad[0];
            Complimentary_Filter.Offset_Rad[1] = Complimentary_Filter.Euler_Angle_Rad[1];
            Complimentary_Filter.Offset_Deg[0] = Complimentary_Filter.Offset_Rad[0] * RAD_TO_DEG;
            Complimentary_Filter.Offset_Deg[1] = Complimentary_Filter.Offset_Rad[1] * RAD_TO_DEG;
        }
    }

    roll_deg = Complimentary_Filter.Euler_Angle_Deg[0];
    pitch_deg = Complimentary_Filter.Euler_Angle_Deg[1];
    yaw_deg = Complimentary_Filter.Euler_Angle_Deg[2];

    App_ToggleHeartbeat(now_ms);

    if ((now_ms - s_last_stream_ms) >= SENSOR_STREAM_MS) {
        s_last_stream_ms = now_ms;
        /*
         * Disable blocking UART logs while profiling the 1 kHz loop.
         * Re-enable this block later if we need serial telemetry again.
         */
        /*
        if (!ICM20602_IsReady()) {
            App_SendLog("ICM-20602 not detected on SPI1\r\n");
            return;
        }

        App_SendLog(
            "Tilt roll=%.2f pitch=%.2f deg | gyro=%.2f,%.2f,%.2f dps\r\n",
            Complimentary_Filter.Euler_Angle_Deg[0],
            Complimentary_Filter.Euler_Angle_Deg[1],
            gyro_phys[0] * RAD_TO_DEG,
            gyro_phys[1] * RAD_TO_DEG,
            gyro_phys[2] * RAD_TO_DEG
        );
        */
    }

    compute_time_us = (uint32_t)(TIM2->CNT - loop_start_us);
    compute_dt_us = compute_time_us;
    if (compute_time_us > max_dt) {
        max_dt = compute_time_us;
    }
    if (compute_time_us > max_compute_dt_us) {
        max_compute_dt_us = compute_time_us;
    }

    while ((uint32_t)(TIM2->CNT - loop_start_us) < APP_TEST_IMU_PERIOD_US) {
    }
}

static void App_RunMagTest(void)
{
    uint32_t now_ms = HAL_GetTick();

    COMPASS_PROCESS();
    App_ToggleHeartbeat(now_ms);

    if ((now_ms - s_last_stream_ms) >= SENSOR_STREAM_MS) {
        s_last_stream_ms = now_ms;
        App_SendLog(
            "MAG raw[uT]=%.2f,%.2f,%.2f cal[uT]=%.2f,%.2f,%.2f\r\n",
            HMC5883L_RAW_DATA.mag[0],
            HMC5883L_RAW_DATA.mag[1],
            HMC5883L_RAW_DATA.mag[2],
            HMC5883L_DATA.mag_uT[0],
            HMC5883L_DATA.mag_uT[1],
            HMC5883L_DATA.mag_uT[2]
        );
    }
}

static void App_SendLog(const char *fmt, ...)
{
    /*
     * Profiling build: keep the formatter stub available, but skip the
     * blocking UART transmit so timing numbers reflect the IMU loop itself.
     */
    (void)fmt;
#if 0
    char buffer[LOG_BUFFER_SIZE];
    int len;
    va_list args;

    if (huart1.Instance != USART1 || huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if (len >= (int)sizeof(buffer)) {
        len = (int)sizeof(buffer) - 1;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, 50U);
#endif
}

static void App_ToggleHeartbeat(uint32_t now_ms)
{
    if ((now_ms - s_last_heartbeat_ms) >= HEARTBEAT_MS) {
        s_last_heartbeat_ms = now_ms;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
