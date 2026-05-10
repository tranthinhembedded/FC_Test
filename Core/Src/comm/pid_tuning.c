#include "comm/pid_tuning.h"

#include "comm/optical_input.h"
#include "control/flight_control.h"
#include "platform/gpio.h"
#include "platform/usart.h"
#include "sensor/sensor_common.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define RX_DMA_SIZE   256
#define CMD_LINE_SIZE 128
#define UART1_ASCII_ACK_ENABLED 0U

static uint8_t rx_dma_buf[RX_DMA_SIZE];
static uint16_t rx_dma_old_pos = 0U;
static char cmd_work[CMD_LINE_SIZE];
static uint16_t cmd_len = 0;
static char cmd_ready[CMD_LINE_SIZE];
#if UART1_ASCII_ACK_ENABLED
static char tx_ack_buf[96];
#endif
static volatile uint8_t line_ready = 0;

static void UART_ParseByte_ISR(uint8_t b);

static void UART1_SendAck(const char *fmt, ...)
{
#if UART1_ASCII_ACK_ENABLED
    va_list args;
    int len;

    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf(tx_ack_buf, sizeof(tx_ack_buf), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if (len >= (int)sizeof(tx_ack_buf)) {
        len = (int)sizeof(tx_ack_buf) - 1;
    }

    (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_ack_buf, (uint16_t)len);
#else
    (void)fmt;
#endif
}

static void UART1_ProcessRxByte(uint8_t byte)
{
    UART_ParseByte_ISR(byte);
}

static void UART1_StartRxDMA(void)
{
    HAL_StatusTypeDef status;

    HAL_UART_DMAStop(&huart1);
    rx_dma_old_pos = 0U;

    status = HAL_UART_Receive_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE);
    if (status != HAL_OK) {
        HAL_UART_DeInit(&huart1);
        HAL_UART_Init(&huart1);
        status = HAL_UART_Receive_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE);
    }

    if (status == HAL_OK) {
        if (huart1.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
        }
    } else {
        OpticalInput_OnUartDmaStartError();
    }
}

static void UART1_ProcessRxDMA_RingBuffer(void)
{
    uint16_t pos;
    uint16_t i;

    /* OpticalInput_OnUartRxPinSample((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1U : 0U); */

    if (huart1.hdmarx == NULL) {
        /* OpticalInput_OnUartDmaState(0U, 0U); */
        return;
    }

    pos = RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (pos == rx_dma_old_pos) {
        return;
    }

    if (pos > rx_dma_old_pos) {
        for (i = rx_dma_old_pos; i < pos; i++) {
            UART1_ProcessRxByte(rx_dma_buf[i]);
        }
    } else {
        for (i = rx_dma_old_pos; i < RX_DMA_SIZE; i++) {
            UART1_ProcessRxByte(rx_dma_buf[i]);
        }
        for (i = 0U; i < pos; i++) {
            UART1_ProcessRxByte(rx_dma_buf[i]);
        }
    }

    rx_dma_old_pos = pos;
}

static void ProcessLine(char *line)
{
    size_t n = strlen(line);

    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == '\n')) {
        line[n - 1] = 0;
        n--;
    }
    if (n == 0) {
        return;
    }

    if (strcmp(line, "MAGCAL") == 0) {
        MagCal.state = MAG_CAL_START;
        MagCal.samples_target = 1000;
        /* Toggle LED to confirm command reception */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(20);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        UART1_SendAck("MSG:MAG CAL START\n");
        return;
    }

    {
        char *tok = strtok(line, ":");
        if (tok && strcmp(tok, "PID") == 0) {
            char *axis = strtok(NULL, ":");
            char *s_kp = strtok(NULL, ":");
            char *s_ki = strtok(NULL, ":");
            char *s_kd = strtok(NULL, ":");

            if (axis && s_kp && s_ki && s_kd) {
                float p = strtof(s_kp, NULL);
                float i = strtof(s_ki, NULL);
                float d = strtof(s_kd, NULL);

                if (strcmp(axis, "RR") == 0) {
                    PID_RATE_ROLL.kp = p; PID_RATE_ROLL.ki = i; PID_RATE_ROLL.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "RP") == 0) {
                    PID_RATE_PITCH.kp = p; PID_RATE_PITCH.ki = i; PID_RATE_PITCH.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "RY") == 0) {
                    PID_RATE_YAW.kp = p; PID_RATE_YAW.ki = i; PID_RATE_YAW.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "AR") == 0) {
                    PID_ROLL.kp = p; PID_ROLL.ki = i; PID_ROLL.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "AP") == 0) {
                    PID_PITCH.kp = p; PID_PITCH.ki = i; PID_PITCH.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "AY") == 0) {
                    PID_YAW.kp = p; PID_YAW.ki = i; PID_YAW.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (axis[0] == 'V' || axis[0] == 'P') {
                    FlightController_SetOpticalFlowPID(axis, p, i, d);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                }

                UART1_SendAck("MSG:UPDATED %s P=%.3f\n", axis, p);
            }
        } else if (tok && strcmp(tok, "OFLOW") == 0) {
            char *s_vx = strtok(NULL, ":");
            char *s_vy = strtok(NULL, ":");
            char *s_alt = strtok(NULL, ":");
            char *s_q = strtok(NULL, ":");
            char *s_dt = strtok(NULL, ":");

            if (s_vx && s_vy && s_alt && s_q && s_dt) {
                float vx = strtof(s_vx, NULL);
                float vy = strtof(s_vy, NULL);
                float alt = strtof(s_alt, NULL);
                float quality = strtof(s_q, NULL);
                float dt = strtof(s_dt, NULL);

                FlightController_UpdateOpticalFlowVelocity(vx, vy, alt, quality, dt);
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

                UART1_SendAck("MSG:OFLOW Vx=%.2f Vy=%.2f Q=%.2f\n", vx, vy, quality);
            }
        } else if (tok && strcmp(tok, "OFRESET") == 0) {
            FlightController_ResetOpticalFlowHold();
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}

static void UART_ParseByte_ISR(uint8_t b)
{
    if (b == '\r') {
        return;
    }
    if (b == '\n') {
        cmd_work[cmd_len] = 0;
        if (!line_ready) {
            strncpy(cmd_ready, cmd_work, CMD_LINE_SIZE);
            cmd_ready[CMD_LINE_SIZE - 1] = 0;
            line_ready = 1;
        }
        cmd_len = 0;
        return;
    }
    if (b >= 32 && b <= 126) {
        if (cmd_len < (CMD_LINE_SIZE - 1)) {
            cmd_work[cmd_len++] = (char)b;
        } else {
            cmd_len = 0;
        }
    }
}

void PidTuning_Init(void)
{
    OpticalInput_Init();
    UART1_StartRxDMA();
}

void PidTuning_ProcessPendingCommand(void)
{
    UART1_ProcessRxDMA_RingBuffer();

    if (line_ready) {
        char local[CMD_LINE_SIZE];

        __disable_irq();
        strncpy(local, cmd_ready, CMD_LINE_SIZE);
        local[CMD_LINE_SIZE - 1] = 0;
        line_ready = 0;
        __enable_irq();

        ProcessLine(local);
    }
}

void PidTuning_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        OpticalInput_OnUartRxEvent(Size);
        UART1_ProcessRxDMA_RingBuffer();
    }
}

void PidTuning_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        volatile uint32_t temp;

        HAL_UART_DMAStop(&huart1);
        temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;
        UART1_StartRxDMA();
    }
}
