#ifndef CORE_APP_COMMUNICATION_OPTICAL_DIRECT_H_
#define CORE_APP_COMMUNICATION_OPTICAL_DIRECT_H_

#include "platform/usart.h"

#include <stdint.h>

extern volatile uint8_t optical_uart6_dma_running;
extern volatile uint32_t optical_uart6_dma_pos;
extern volatile uint32_t optical_uart6_dma_start_error_count;
extern volatile uint32_t optical_uart6_dma_restart_count;
extern volatile uint32_t optical_uart6_error_count;
extern volatile uint32_t optical_uart6_last_error_code;

extern volatile uint32_t optical_uart6_byte_count;
extern volatile uint8_t optical_uart6_last_byte;
extern volatile uint8_t optical_uart6_recent_bytes[16];
extern volatile uint8_t optical_uart6_recent_index;
extern volatile uint32_t optical_uart6_pending_bytes;
extern volatile uint32_t optical_uart6_max_pending_bytes;
extern volatile uint32_t optical_uart6_processed_bytes_last;
extern volatile uint32_t optical_uart6_process_limit_count;

extern volatile uint8_t optical_uart6_rx_pin_level;
extern volatile uint32_t optical_uart6_rx_pin_edge_count;
extern volatile uint32_t optical_uart6_rx_pin_low_count;

extern volatile uint32_t optical_uart6_msp_frame_count;
extern volatile uint32_t optical_uart6_msp_crc_error_count;
extern volatile uint32_t optical_uart6_micolink_frame_count;
extern volatile uint32_t optical_uart6_micolink_checksum_error_count;
extern volatile uint32_t optical_uart6_unknown_frame_count;

void OpticalDirect_Init(void);
void OpticalDirect_Process(void);
void OpticalDirect_HandleUartError(UART_HandleTypeDef *huart);

#endif
