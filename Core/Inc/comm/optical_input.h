#ifndef CORE_APP_COMMUNICATION_OPTICAL_INPUT_H_
#define CORE_APP_COMMUNICATION_OPTICAL_INPUT_H_

#include "comm/optical_packet.h"

#include <stdint.h>

extern volatile uint8_t optical_rx_packet_valid;
extern volatile uint8_t optical_rx_seq;
extern volatile uint32_t optical_rx_timestamp_ms;
extern volatile uint16_t optical_rx_flags;
extern volatile int32_t optical_rx_distance_mm;
extern volatile int32_t optical_rx_flow_x_raw;
extern volatile int32_t optical_rx_flow_y_raw;
extern volatile uint8_t optical_rx_range_quality;
extern volatile uint8_t optical_rx_flow_quality;

extern volatile uint8_t optical_rx_distance_valid;
extern volatile uint8_t optical_rx_flow_valid;
extern volatile uint8_t optical_rx_range_quality_valid;
extern volatile uint8_t optical_rx_flow_quality_valid;
extern volatile uint8_t optical_rx_distance_out_of_range;
extern volatile uint8_t optical_rx_protocol_msp;
extern volatile uint8_t optical_rx_protocol_micolink;

extern volatile uint8_t optical_rx_parser_index;
extern volatile uint32_t optical_rx_frame_count;
extern volatile uint32_t optical_rx_seq_gap_count;
extern volatile uint32_t optical_rx_last_frame_tick_ms;
extern volatile uint32_t optical_rx_byte_count;
extern volatile uint32_t optical_rx_sync1_count;
extern volatile uint8_t optical_rx_last_byte;
extern volatile uint8_t optical_rx_recent_bytes[16];
extern volatile uint8_t optical_rx_recent_index;

extern volatile uint32_t optical_uart1_rx_event_count;
extern volatile uint16_t optical_uart1_rx_last_size;
extern volatile uint32_t optical_uart1_error_count;
extern volatile uint32_t optical_uart1_last_error_code;
extern volatile uint32_t optical_uart1_dma_pos;
extern volatile uint32_t optical_uart1_dma_start_error_count;
extern volatile uint32_t optical_uart1_dma_restart_count;
extern volatile uint8_t optical_uart1_dma_running;
extern volatile uint8_t optical_uart1_rx_pin_level;
extern volatile uint32_t optical_uart1_rx_pin_edge_count;
extern volatile uint32_t optical_uart1_rx_pin_low_count;

void OpticalInput_Init(void);
void OpticalInput_ApplyPayload(uint8_t seq, const optical_payload_t *payload);
uint8_t OpticalInput_FeedByte(uint8_t byte);
void OpticalInput_OnUartRxEvent(uint16_t size);
void OpticalInput_OnUartError(uint32_t error_code);
void OpticalInput_OnUartDmaState(uint16_t pos, uint8_t running);
void OpticalInput_OnUartDmaStartError(void);
void OpticalInput_OnUartDmaRestart(void);
void OpticalInput_OnUartRxPinSample(uint8_t level);

#endif
