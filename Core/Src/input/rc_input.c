#include "main.h"
#include "input/rc_input.h"
#include "sensor/sensor_common.h"
#include "platform/usart.h"

/* iBUS protocol definitions */
#define IBUS_DMA_BUFFER_SIZE 64
#define IBUS_FRAME_SIZE 32U
#define IBUS_FRAME_LEN 0x20U
#define IBUS_COMMAND_SERVO 0x40U
#define RC_LINK_TIMEOUT_MS 100U

static uint8_t ibus_dma_buf[IBUS_DMA_BUFFER_SIZE];
static uint16_t ibus_old_pos = 0;

static uint8_t ibus_state_idx = 0;
static uint8_t ibus_frame_buf[32];

volatile uint32_t rc_last_frame_tick_ms = 0U;
volatile uint32_t rc_valid_frame_count = 0U;
volatile uint8_t rc_link_ok = 0U;
volatile uint32_t rc_link_drop_count = 0U;
volatile uint32_t rc_uart_error_count = 0U;
volatile uint32_t rc_last_error_tick_ms = 0U;
volatile uint32_t rc_last_timeout_tick_ms = 0U;
volatile uint32_t rc_dma_pos = 0U;
volatile uint32_t rc_byte_count = 0U;
volatile uint32_t rc_frame_sync_count = 0U;
volatile uint32_t rc_checksum_error_count = 0U;
volatile uint32_t rc_bad_header_count = 0U;
volatile uint32_t rc_dma_start_error_count = 0U;
volatile uint8_t rc_dma_running = 0U;

static void RcReceiver_ApplyFailsafeValues(void)
{
    RC_Raw_Roll = 1500U;
    RC_Raw_Pitch = 1500U;
    RC_Raw_Yaw = 1500U;
    RC_Raw_Throttle = 1000U;
    RC_Raw_SW_Arm = 1000U;
    RC_Raw_SW_Mode = 1000U;
}

void RcReceiver_Init(void)
{
    HAL_StatusTypeDef dma_status;

    RcReceiver_ApplyFailsafeValues();

    /* Request DMA circular reception */
    ibus_old_pos = 0;
    ibus_state_idx = 0U;
    rc_last_frame_tick_ms = 0U;
    rc_valid_frame_count = 0U;
    rc_link_ok = 0U;
    rc_link_drop_count = 0U;
    rc_uart_error_count = 0U;
    rc_last_error_tick_ms = 0U;
    rc_last_timeout_tick_ms = 0U;
    rc_dma_pos = 0U;
    rc_byte_count = 0U;
    rc_frame_sync_count = 0U;
    rc_checksum_error_count = 0U;
    rc_bad_header_count = 0U;
    rc_dma_start_error_count = 0U;
    rc_dma_running = 0U;

    dma_status = HAL_UART_Receive_DMA(&huart2, ibus_dma_buf, IBUS_DMA_BUFFER_SIZE);
    if (dma_status == HAL_OK) {
        rc_dma_running = 1U;
    } else {
        rc_dma_start_error_count++;
    }
}

/* Parse a single byte through the iBUS state machine */
static void Process_iBUS_Byte(uint8_t byte)
{
    if (ibus_state_idx == 0) {
        if (byte == IBUS_FRAME_LEN) {
            ibus_frame_buf[ibus_state_idx++] = byte;
        } else {
            rc_bad_header_count++;
        }
    } 
    else if (ibus_state_idx == 1) {
        if (byte == IBUS_COMMAND_SERVO) {
            ibus_frame_buf[ibus_state_idx++] = byte;
        } else {
            rc_bad_header_count++;
            if (byte == IBUS_FRAME_LEN) {
                ibus_frame_buf[0] = byte;
                ibus_state_idx = 1U;
            } else {
                ibus_state_idx = 0U;
            }
        }
    } 
    else {
        ibus_frame_buf[ibus_state_idx++] = byte;
        
        if (ibus_state_idx == IBUS_FRAME_SIZE) {
            /* Checksum validation */
            uint16_t checksum = 0xFFFF;
            for (int i = 0; i < 30; i++) {
                checksum -= ibus_frame_buf[i];
            }
            
            uint16_t received_checksum = (uint16_t)ibus_frame_buf[30] | ((uint16_t)ibus_frame_buf[31] << 8);

            rc_frame_sync_count++;
            
            if (checksum == received_checksum) {
                /* Valid frame received, decode channels */
                RC_Raw_Roll     = (uint32_t)((uint16_t)ibus_frame_buf[2]  | ((uint16_t)ibus_frame_buf[3]  << 8));
                RC_Raw_Pitch    = (uint32_t)((uint16_t)ibus_frame_buf[4]  | ((uint16_t)ibus_frame_buf[5]  << 8));
                RC_Raw_Throttle = (uint32_t)((uint16_t)ibus_frame_buf[6]  | ((uint16_t)ibus_frame_buf[7]  << 8));
                RC_Raw_Yaw      = (uint32_t)((uint16_t)ibus_frame_buf[8]  | ((uint16_t)ibus_frame_buf[9]  << 8));
                RC_Raw_SW_Arm   = (uint32_t)((uint16_t)ibus_frame_buf[10] | ((uint16_t)ibus_frame_buf[11] << 8));
                RC_Raw_SW_Mode  = (uint32_t)((uint16_t)ibus_frame_buf[12] | ((uint16_t)ibus_frame_buf[13] << 8));
                rc_last_frame_tick_ms = HAL_GetTick();
                rc_valid_frame_count++;
                rc_link_ok = 1U;
            } else {
                rc_checksum_error_count++;
            }
            
            ibus_state_idx = 0; /* Reset for next frame */
        }
    }
}

/* Calculate available bytes from DMA buffer and process */
void RcReceiver_Process_DMA_Ring_Buffer(void)
{
    if (huart2.hdmarx == NULL) {
        rc_dma_running = 0U;
        return;
    }

    uint16_t pos = IBUS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    rc_dma_pos = pos;
    
    if (pos != ibus_old_pos) {
        if (pos > ibus_old_pos) {
            /* Data hasn't wrapped around the buffer end */
            for (uint16_t i = ibus_old_pos; i < pos; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
        } 
        else {
            /* Data wrapped around to the beginning */
            for (uint16_t i = ibus_old_pos; i < IBUS_DMA_BUFFER_SIZE; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                rc_byte_count++;
                Process_iBUS_Byte(ibus_dma_buf[i]);
            }
        }
        ibus_old_pos = pos;
    }
}

void RcReceiver_UpdateLinkStatus(uint32_t now_ms)
{
    if ((rc_last_frame_tick_ms == 0U) || ((uint32_t)(now_ms - rc_last_frame_tick_ms) > RC_LINK_TIMEOUT_MS)) {
        if (rc_link_ok != 0U) {
            rc_link_drop_count++;
            rc_last_timeout_tick_ms = now_ms;
        }
        rc_link_ok = 0U;
        RcReceiver_ApplyFailsafeValues();
        return;
    }

    rc_link_ok = 1U;
}

void RcReceiver_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_StatusTypeDef dma_status;

        /* Clear error flags */
        volatile uint32_t temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;

        rc_uart_error_count++;
        rc_last_error_tick_ms = HAL_GetTick();
        ibus_state_idx = 0U;
        ibus_old_pos = 0U;

        /* Restart DMA */
        HAL_UART_DMAStop(&huart2);
        dma_status = HAL_UART_Receive_DMA(&huart2, ibus_dma_buf, IBUS_DMA_BUFFER_SIZE);
        if (dma_status == HAL_OK) {
            rc_dma_running = 1U;
        } else {
            rc_dma_running = 0U;
            rc_dma_start_error_count++;
        }
    }
}
