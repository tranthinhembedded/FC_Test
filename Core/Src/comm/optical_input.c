#include "comm/optical_input.h"

#include "main.h"

volatile uint8_t optical_rx_packet_valid = 0U;
volatile uint8_t optical_rx_seq = 0U;
volatile uint32_t optical_rx_timestamp_ms = 0U;
volatile uint16_t optical_rx_flags = 0U;
volatile int32_t optical_rx_distance_mm = 0;
volatile int32_t optical_rx_flow_x_raw = 0;
volatile int32_t optical_rx_flow_y_raw = 0;
volatile uint8_t optical_rx_range_quality = 0U;
volatile uint8_t optical_rx_flow_quality = 0U;

volatile uint8_t optical_rx_distance_valid = 0U;
volatile uint8_t optical_rx_flow_valid = 0U;
volatile uint8_t optical_rx_range_quality_valid = 0U;
volatile uint8_t optical_rx_flow_quality_valid = 0U;
volatile uint8_t optical_rx_distance_out_of_range = 0U;
volatile uint8_t optical_rx_protocol_msp = 0U;
volatile uint8_t optical_rx_protocol_micolink = 0U;

volatile uint8_t optical_rx_parser_index = 0U;
volatile uint32_t optical_rx_frame_count = 0U;
volatile uint32_t optical_rx_seq_gap_count = 0U;
volatile uint32_t optical_rx_last_frame_tick_ms = 0U;
volatile uint32_t optical_rx_byte_count = 0U;
volatile uint32_t optical_rx_sync1_count = 0U;
volatile uint8_t optical_rx_last_byte = 0U;
volatile uint8_t optical_rx_recent_bytes[16] = {0U};
volatile uint8_t optical_rx_recent_index = 0U;

volatile uint32_t optical_uart1_rx_event_count = 0U;
volatile uint16_t optical_uart1_rx_last_size = 0U;
volatile uint32_t optical_uart1_error_count = 0U;
volatile uint32_t optical_uart1_last_error_code = 0U;
volatile uint32_t optical_uart1_dma_pos = 0U;
volatile uint32_t optical_uart1_dma_start_error_count = 0U;
volatile uint32_t optical_uart1_dma_restart_count = 0U;
volatile uint8_t optical_uart1_dma_running = 0U;
volatile uint8_t optical_uart1_rx_pin_level = 1U;
volatile uint32_t optical_uart1_rx_pin_edge_count = 0U;
volatile uint32_t optical_uart1_rx_pin_low_count = 0U;

static optical_parser_t s_optical_parser;
static uint8_t s_optical_seq_initialized = 0U;
static uint8_t s_optical_last_seq = 0U;

static void OpticalInput_UpdateFlags(uint16_t flags)
{
    optical_rx_distance_valid = (flags & OPTICAL_FLAG_DISTANCE_VALID) ? 1U : 0U;
    optical_rx_flow_valid = (flags & OPTICAL_FLAG_FLOW_VALID) ? 1U : 0U;
    optical_rx_range_quality_valid = (flags & OPTICAL_FLAG_RANGE_QUALITY_VALID) ? 1U : 0U;
    optical_rx_flow_quality_valid = (flags & OPTICAL_FLAG_FLOW_QUALITY_VALID) ? 1U : 0U;
    optical_rx_distance_out_of_range = (flags & OPTICAL_FLAG_DISTANCE_OUT_RANGE) ? 1U : 0U;
    optical_rx_protocol_msp = (flags & OPTICAL_FLAG_PROTOCOL_MSP) ? 1U : 0U;
    optical_rx_protocol_micolink = (flags & OPTICAL_FLAG_PROTOCOL_MICOLINK) ? 1U : 0U;
}

void OpticalInput_Init(void)
{
    optical_parser_init(&s_optical_parser);
    s_optical_seq_initialized = 0U;
    s_optical_last_seq = 0U;

    optical_rx_packet_valid = 0U;
    optical_rx_seq = 0U;
    optical_rx_timestamp_ms = 0U;
    optical_rx_flags = 0U;
    optical_rx_distance_mm = 0;
    optical_rx_flow_x_raw = 0;
    optical_rx_flow_y_raw = 0;
    optical_rx_range_quality = 0U;
    optical_rx_flow_quality = 0U;
    optical_rx_distance_valid = 0U;
    optical_rx_flow_valid = 0U;
    optical_rx_range_quality_valid = 0U;
    optical_rx_flow_quality_valid = 0U;
    optical_rx_distance_out_of_range = 0U;
    optical_rx_protocol_msp = 0U;
    optical_rx_protocol_micolink = 0U;
    optical_rx_parser_index = 0U;
    optical_rx_frame_count = 0U;
    optical_rx_seq_gap_count = 0U;
    optical_rx_last_frame_tick_ms = 0U;
    optical_rx_byte_count = 0U;
    optical_rx_sync1_count = 0U;
    optical_rx_last_byte = 0U;
    optical_rx_recent_index = 0U;
    for (uint8_t i = 0U; i < 16U; i++) {
        optical_rx_recent_bytes[i] = 0U;
    }
    optical_uart1_rx_event_count = 0U;
    optical_uart1_rx_last_size = 0U;
    optical_uart1_error_count = 0U;
    optical_uart1_last_error_code = 0U;
    optical_uart1_dma_pos = 0U;
    optical_uart1_dma_start_error_count = 0U;
    optical_uart1_dma_restart_count = 0U;
    optical_uart1_dma_running = 0U;
    optical_uart1_rx_pin_level = 1U;
    optical_uart1_rx_pin_edge_count = 0U;
    optical_uart1_rx_pin_low_count = 0U;
}

void OpticalInput_OnUartRxEvent(uint16_t size)
{
    optical_uart1_rx_event_count++;
    optical_uart1_rx_last_size = size;
}

void OpticalInput_OnUartError(uint32_t error_code)
{
    optical_uart1_error_count++;
    optical_uart1_last_error_code = error_code;
}

void OpticalInput_OnUartDmaState(uint16_t pos, uint8_t running)
{
    optical_uart1_dma_pos = pos;
    optical_uart1_dma_running = running;
}

void OpticalInput_OnUartDmaStartError(void)
{
    optical_uart1_dma_start_error_count++;
    optical_uart1_dma_running = 0U;
}

void OpticalInput_OnUartDmaRestart(void)
{
    optical_uart1_dma_restart_count++;
}

void OpticalInput_OnUartRxPinSample(uint8_t level)
{
    level = (level != 0U) ? 1U : 0U;
    if (level != optical_uart1_rx_pin_level) {
        optical_uart1_rx_pin_edge_count++;
    }
    optical_uart1_rx_pin_level = level;
    if (level == 0U) {
        optical_uart1_rx_pin_low_count++;
    }
}

void OpticalInput_ApplyPayload(uint8_t seq, const optical_payload_t *payload)
{
    optical_rx_packet_valid = 1U;
    optical_rx_seq = seq;
    optical_rx_timestamp_ms = payload->timestamp_ms;
    optical_rx_flags = payload->flags;
    optical_rx_distance_mm = payload->distance_mm;
    optical_rx_flow_x_raw = payload->flow_x_raw;
    optical_rx_flow_y_raw = payload->flow_y_raw;
    optical_rx_range_quality = payload->range_quality;
    optical_rx_flow_quality = payload->flow_quality;
    optical_rx_last_frame_tick_ms = HAL_GetTick();
    optical_rx_frame_count++;
    OpticalInput_UpdateFlags(payload->flags);

    if (s_optical_seq_initialized != 0U) {
        if ((uint8_t)(s_optical_last_seq + 1U) != seq) {
            optical_rx_seq_gap_count++;
        }
    }
    s_optical_seq_initialized = 1U;
    s_optical_last_seq = seq;
}

uint8_t OpticalInput_FeedByte(uint8_t byte)
{
    optical_packet_t packet;
    uint8_t parser_index_before;
    uint8_t consumed;

    optical_rx_byte_count++;
    optical_rx_last_byte = byte;
    optical_rx_recent_bytes[optical_rx_recent_index] = byte;
    optical_rx_recent_index = (uint8_t)((optical_rx_recent_index + 1U) & 0x0FU);
    if (byte == OPTICAL_SYNC1) {
        optical_rx_sync1_count++;
    }

    parser_index_before = s_optical_parser.index;
    consumed = ((parser_index_before != 0U) || (byte == OPTICAL_SYNC1)) ? 1U : 0U;

    if (optical_packet_feed(&s_optical_parser, byte, &packet) != 0) {
        OpticalInput_ApplyPayload(packet.seq, &packet.payload);
        optical_rx_parser_index = s_optical_parser.index;
        return 1U;
    }

    optical_rx_parser_index = s_optical_parser.index;
    if ((consumed != 0U) || (s_optical_parser.index != 0U)) {
        return 1U;
    }

    return 0U;
}
