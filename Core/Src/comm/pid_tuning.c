#include "comm/pid_tuning.h"

#include "platform/gpio.h"
#include "platform/usart.h"
#include "sensor/sensor_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_DMA_SIZE   256
#define CMD_LINE_SIZE 128

static uint8_t rx_dma_buf[RX_DMA_SIZE];
static char cmd_work[CMD_LINE_SIZE];
static uint16_t cmd_len = 0;
static char cmd_ready[CMD_LINE_SIZE];
static volatile uint8_t line_ready = 0;

static void UART1_StartRxToIdle_DMA(void)
{
    HAL_UART_DMAStop(&huart1);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE) != HAL_OK) {
        HAL_UART_DeInit(&huart1);
        HAL_UART_Init(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_SIZE);
    }
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
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

                if (strcmp(axis, "ANG") == 0) {
                    PID_ROLL.kp = p;
                    PID_ROLL.ki = i;
                    PID_ROLL.kd = d;
                    PID_PITCH.kp = p;
                    PID_PITCH.ki = i;
                    PID_PITCH.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "YANG") == 0) {
                    PID_YAW.kp = p;
                    PID_YAW.ki = i;
                    PID_YAW.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                } else if (strcmp(axis, "YAW") == 0) {
                    PID_RATE_YAW.kp = p;
                    PID_RATE_YAW.ki = i;
                    PID_RATE_YAW.kd = d;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                }

                if (huart1.gState == HAL_UART_STATE_READY) {
                    char ack[64];
                    int len = snprintf(ack, sizeof(ack), "MSG:UPDATED %s P=%.3f\n", axis, p);

                    if (len > 0) {
                        HAL_UART_Transmit(&huart1, (uint8_t *)ack, (uint16_t)len, 10);
                    }
                }
            }
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
    UART1_StartRxToIdle_DMA();
}

void PidTuning_ProcessPendingCommand(void)
{
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        uint16_t i;

        for (i = 0; i < Size; i++) {
            UART_ParseByte_ISR(rx_dma_buf[i]);
        }
        UART1_StartRxToIdle_DMA();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        volatile uint32_t temp;

        HAL_UART_DMAStop(&huart1);
        temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;
        UART1_StartRxToIdle_DMA();
    }
}
