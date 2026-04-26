#ifndef CORE_APP_RC_RECEIVER_RECEIVER_H_
#define CORE_APP_RC_RECEIVER_RECEIVER_H_

#include <stdint.h>
#include "platform/usart.h"

extern volatile uint32_t rc_last_frame_tick_ms;
extern volatile uint32_t rc_valid_frame_count;
extern volatile uint8_t rc_link_ok;
extern volatile uint32_t rc_link_drop_count;
extern volatile uint32_t rc_uart_error_count;
extern volatile uint32_t rc_last_error_tick_ms;
extern volatile uint32_t rc_last_timeout_tick_ms;
extern volatile uint32_t rc_dma_pos;
extern volatile uint32_t rc_byte_count;
extern volatile uint32_t rc_frame_sync_count;
extern volatile uint32_t rc_checksum_error_count;
extern volatile uint32_t rc_bad_header_count;
extern volatile uint32_t rc_dma_start_error_count;
extern volatile uint8_t rc_dma_running;

void RcReceiver_Init(void);
void RcReceiver_Process_DMA_Ring_Buffer(void);
void RcReceiver_UpdateLinkStatus(uint32_t now_ms);
void RcReceiver_HandleUartError(UART_HandleTypeDef *huart);

#endif /* CORE_APP_RC_RECEIVER_RECEIVER_H_ */
