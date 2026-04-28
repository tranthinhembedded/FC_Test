#include "comm/optical_direct.h"

#include "comm/optical_input.h"
#include "main.h"

#define OPTICAL_DIRECT_DMA_SIZE 256U
#define OPTICAL_DIRECT_RECENT_SIZE 16U
#define OPTICAL_DIRECT_MAX_BYTES_PER_CALL 16U
#define OPTICAL_DIRECT_RX_PIN_SAMPLE_DIV 10U

#define MICOLINK_HEAD 0xEFU
#define MICOLINK_MAX_PAYLOAD_LEN 64U
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51U
#define MICOLINK_PAYLOAD_LEN 20U

#define MSP_V2_MAX_PAYLOAD_LEN 256U
#define MSP2_SENSOR_RANGEFINDER 0x1F01U
#define MSP2_SENSOR_OPTIC_FLOW 0x1F02U

volatile uint8_t optical_uart6_dma_running = 0U;
volatile uint32_t optical_uart6_dma_pos = 0U;
volatile uint32_t optical_uart6_dma_start_error_count = 0U;
volatile uint32_t optical_uart6_dma_restart_count = 0U;
volatile uint32_t optical_uart6_error_count = 0U;
volatile uint32_t optical_uart6_last_error_code = 0U;

volatile uint32_t optical_uart6_byte_count = 0U;
volatile uint8_t optical_uart6_last_byte = 0U;
volatile uint8_t optical_uart6_recent_bytes[OPTICAL_DIRECT_RECENT_SIZE] = {0U};
volatile uint8_t optical_uart6_recent_index = 0U;
volatile uint32_t optical_uart6_pending_bytes = 0U;
volatile uint32_t optical_uart6_max_pending_bytes = 0U;
volatile uint32_t optical_uart6_processed_bytes_last = 0U;
volatile uint32_t optical_uart6_process_limit_count = 0U;

volatile uint8_t optical_uart6_rx_pin_level = 1U;
volatile uint32_t optical_uart6_rx_pin_edge_count = 0U;
volatile uint32_t optical_uart6_rx_pin_low_count = 0U;

volatile uint32_t optical_uart6_msp_frame_count = 0U;
volatile uint32_t optical_uart6_msp_crc_error_count = 0U;
volatile uint32_t optical_uart6_micolink_frame_count = 0U;
volatile uint32_t optical_uart6_micolink_checksum_error_count = 0U;
volatile uint32_t optical_uart6_unknown_frame_count = 0U;

static uint8_t s_uart6_dma_buf[OPTICAL_DIRECT_DMA_SIZE];
static uint16_t s_uart6_dma_old_pos = 0U;
static uint8_t s_rx_pin_sample_div = OPTICAL_DIRECT_RX_PIN_SAMPLE_DIV - 1U;

static uint8_t s_micolink_buf[6U + MICOLINK_MAX_PAYLOAD_LEN + 1U];
static uint8_t s_micolink_index = 0U;
static uint8_t s_micolink_payload_len = 0U;

static uint8_t s_msp_buf[9U + MSP_V2_MAX_PAYLOAD_LEN];
static uint16_t s_msp_index = 0U;
static uint16_t s_msp_payload_len = 0U;

static uint8_t s_direct_seq = 0U;
static int32_t s_msp_distance_mm = 0;
static uint8_t s_msp_distance_valid = 0U;
static uint8_t s_msp_range_quality = 0U;
static uint8_t s_msp_range_quality_valid = 0U;
static int32_t s_msp_flow_x_raw = 0;
static int32_t s_msp_flow_y_raw = 0;
static uint8_t s_msp_flow_valid = 0U;
static uint8_t s_msp_flow_quality = 0U;
static uint8_t s_msp_flow_quality_valid = 0U;
static uint8_t s_msp_seen_flow_frame = 0U;

static uint16_t optical_direct_pending_from_pos(uint16_t pos)
{
    if (pos >= s_uart6_dma_old_pos) {
        return (uint16_t)(pos - s_uart6_dma_old_pos);
    }

    return (uint16_t)(OPTICAL_DIRECT_DMA_SIZE - s_uart6_dma_old_pos + pos);
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)read_u16_le(p);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
}

static uint8_t optical_direct_crc8_dvb_s2(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0U;

    for (uint16_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

static void optical_direct_reset_micolink(void)
{
    s_micolink_index = 0U;
    s_micolink_payload_len = 0U;
}

static void optical_direct_reset_msp(void)
{
    s_msp_index = 0U;
    s_msp_payload_len = 0U;
}

static void optical_direct_apply_msp_state(void)
{
    optical_payload_t payload = {0};

    payload.timestamp_ms = HAL_GetTick();
    payload.flags = OPTICAL_FLAG_PROTOCOL_MSP;

    if ((s_msp_distance_valid != 0U) && (s_msp_distance_mm > 0)) {
        payload.flags |= OPTICAL_FLAG_DISTANCE_VALID;
        payload.distance_mm = s_msp_distance_mm;
    } else {
        payload.flags |= OPTICAL_FLAG_DISTANCE_OUT_RANGE;
    }

    if (s_msp_range_quality_valid != 0U) {
        payload.flags |= OPTICAL_FLAG_RANGE_QUALITY_VALID;
        payload.range_quality = s_msp_range_quality;
    }

    if (s_msp_flow_valid != 0U) {
        payload.flags |= OPTICAL_FLAG_FLOW_VALID;
        payload.flow_x_raw = s_msp_flow_x_raw;
        payload.flow_y_raw = s_msp_flow_y_raw;
    }

    if (s_msp_flow_quality_valid != 0U) {
        payload.flags |= OPTICAL_FLAG_FLOW_QUALITY_VALID;
        payload.flow_quality = s_msp_flow_quality;
    }

    OpticalInput_ApplyPayload(s_direct_seq++, &payload);
}

static void optical_direct_handle_micolink_frame(const uint8_t *raw, uint16_t len)
{
    uint8_t sum = 0U;
    const uint8_t *payload;
    uint32_t distance_mm_u32;
    uint8_t distance_status;
    uint8_t flow_status;
    optical_payload_t out = {0};

    for (uint16_t i = 0U; i < (uint16_t)(len - 1U); i++) {
        sum = (uint8_t)(sum + raw[i]);
    }
    if (sum != raw[len - 1U]) {
        optical_uart6_micolink_checksum_error_count++;
        return;
    }

    if ((raw[3] != MICOLINK_MSG_ID_RANGE_SENSOR) || (raw[5] < MICOLINK_PAYLOAD_LEN)) {
        optical_uart6_unknown_frame_count++;
        return;
    }

    payload = &raw[6];
    distance_mm_u32 = read_u32_le(&payload[4]);
    distance_status = payload[10];
    flow_status = payload[17];

    out.timestamp_ms = HAL_GetTick();
    out.flags = OPTICAL_FLAG_PROTOCOL_MICOLINK | OPTICAL_FLAG_RANGE_QUALITY_VALID;
    out.range_quality = payload[8];

    if ((distance_mm_u32 > 0U) && (distance_status == 0U)) {
        out.flags |= OPTICAL_FLAG_DISTANCE_VALID;
        out.distance_mm = (int32_t)distance_mm_u32;
    } else {
        out.flags |= OPTICAL_FLAG_DISTANCE_OUT_RANGE;
    }

    if (flow_status == 0U) {
        out.flags |= OPTICAL_FLAG_FLOW_VALID | OPTICAL_FLAG_FLOW_QUALITY_VALID;
        out.flow_x_raw = (int32_t)read_i16_le(&payload[12]);
        out.flow_y_raw = (int32_t)read_i16_le(&payload[14]);
        out.flow_quality = payload[16];
    }

    optical_uart6_micolink_frame_count++;
    OpticalInput_ApplyPayload(raw[4], &out);
}

static void optical_direct_handle_msp_frame(const uint8_t *raw, uint16_t len)
{
    uint16_t command;
    uint16_t payload_len;
    const uint8_t *payload;
    uint8_t crc;

    crc = optical_direct_crc8_dvb_s2(&raw[3], (uint16_t)(len - 4U));
    if (crc != raw[len - 1U]) {
        optical_uart6_msp_crc_error_count++;
        return;
    }

    command = read_u16_le(&raw[4]);
    payload_len = read_u16_le(&raw[6]);
    payload = &raw[8];

    if ((command == MSP2_SENSOR_RANGEFINDER) && (payload_len >= 5U)) {
        int32_t distance_mm = read_i32_le(&payload[1]);

        s_msp_range_quality = payload[0];
        s_msp_range_quality_valid = 1U;
        if (distance_mm >= 0) {
            s_msp_distance_mm = distance_mm;
            s_msp_distance_valid = 1U;
        } else {
            s_msp_distance_mm = 0;
            s_msp_distance_valid = 0U;
        }
        optical_uart6_msp_frame_count++;
        if (s_msp_seen_flow_frame == 0U) {
            optical_direct_apply_msp_state();
        }
        return;
    }

    if ((command == MSP2_SENSOR_OPTIC_FLOW) && (payload_len >= 9U)) {
        s_msp_flow_quality = payload[0];
        s_msp_flow_quality_valid = 1U;
        s_msp_flow_x_raw = read_i32_le(&payload[1]);
        s_msp_flow_y_raw = read_i32_le(&payload[5]);
        s_msp_flow_valid = 1U;
        optical_uart6_msp_frame_count++;
        s_msp_seen_flow_frame = 1U;
        optical_direct_apply_msp_state();
        return;
    }

    optical_uart6_unknown_frame_count++;
}

static void optical_direct_feed_micolink(uint8_t byte)
{
    uint16_t full_len;

    if (s_micolink_index == 0U) {
        if (byte == MICOLINK_HEAD) {
            s_micolink_buf[s_micolink_index++] = byte;
        }
        return;
    }

    s_micolink_buf[s_micolink_index++] = byte;

    if (s_micolink_index == 6U) {
        s_micolink_payload_len = s_micolink_buf[5];
        if (s_micolink_payload_len > MICOLINK_MAX_PAYLOAD_LEN) {
            optical_direct_reset_micolink();
        }
        return;
    }

    full_len = (uint16_t)(6U + s_micolink_payload_len + 1U);
    if (s_micolink_index < full_len) {
        return;
    }

    optical_direct_handle_micolink_frame(s_micolink_buf, full_len);
    optical_direct_reset_micolink();
}

static void optical_direct_msp_restart_or_reset(uint8_t byte)
{
    optical_direct_reset_msp();
    if (byte == (uint8_t)'$') {
        s_msp_buf[s_msp_index++] = byte;
    }
}

static void optical_direct_feed_msp(uint8_t byte)
{
    uint16_t full_len;

    if (s_msp_index == 0U) {
        if (byte == (uint8_t)'$') {
            s_msp_buf[s_msp_index++] = byte;
        }
        return;
    }

    s_msp_buf[s_msp_index++] = byte;

    if ((s_msp_index == 2U) && (s_msp_buf[1] != (uint8_t)'X')) {
        optical_direct_msp_restart_or_reset(byte);
        return;
    }

    if ((s_msp_index == 3U)
        && (s_msp_buf[2] != (uint8_t)'<')
        && (s_msp_buf[2] != (uint8_t)'>')
        && (s_msp_buf[2] != (uint8_t)'!')) {
        optical_direct_msp_restart_or_reset(byte);
        return;
    }

    if (s_msp_index == 8U) {
        s_msp_payload_len = read_u16_le(&s_msp_buf[6]);
        if (s_msp_payload_len > MSP_V2_MAX_PAYLOAD_LEN) {
            optical_direct_reset_msp();
        }
        return;
    }

    full_len = (uint16_t)(9U + s_msp_payload_len);
    if (s_msp_index < full_len) {
        return;
    }

    optical_direct_handle_msp_frame(s_msp_buf, full_len);
    optical_direct_reset_msp();
}

static void optical_direct_record_byte(uint8_t byte)
{
    optical_uart6_byte_count++;
    optical_uart6_last_byte = byte;
    optical_uart6_recent_bytes[optical_uart6_recent_index] = byte;
    optical_uart6_recent_index = (uint8_t)((optical_uart6_recent_index + 1U) & 0x0FU);

    optical_rx_byte_count++;
    optical_rx_last_byte = byte;
    optical_rx_recent_bytes[optical_rx_recent_index] = byte;
    optical_rx_recent_index = (uint8_t)((optical_rx_recent_index + 1U) & 0x0FU);

    if (s_msp_index != 0U) {
        optical_direct_feed_msp(byte);
    } else if (s_micolink_index != 0U) {
        optical_direct_feed_micolink(byte);
    } else if (byte == (uint8_t)'$') {
        optical_direct_feed_msp(byte);
    } else if (byte == MICOLINK_HEAD) {
        optical_direct_feed_micolink(byte);
    }
}

static void optical_direct_sample_rx_pin(void)
{
    uint8_t level = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) ? 1U : 0U;

    if (level != optical_uart6_rx_pin_level) {
        optical_uart6_rx_pin_edge_count++;
    }
    optical_uart6_rx_pin_level = level;
    if (level == 0U) {
        optical_uart6_rx_pin_low_count++;
    }
}

static void optical_direct_start_rx_dma(void)
{
    HAL_StatusTypeDef status;

    HAL_UART_DMAStop(&huart6);
    s_uart6_dma_old_pos = 0U;
    optical_uart6_dma_pos = 0U;
    optical_uart6_dma_running = 0U;

    status = HAL_UART_Receive_DMA(&huart6, s_uart6_dma_buf, OPTICAL_DIRECT_DMA_SIZE);
    if (status != HAL_OK) {
        HAL_UART_DeInit(&huart6);
        HAL_UART_Init(&huart6);
        status = HAL_UART_Receive_DMA(&huart6, s_uart6_dma_buf, OPTICAL_DIRECT_DMA_SIZE);
    }

    if (status == HAL_OK) {
        optical_uart6_dma_running = 1U;
        if (huart6.hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
            __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_TC);
        }
    } else {
        optical_uart6_dma_start_error_count++;
    }
}

void OpticalDirect_Init(void)
{
    OpticalInput_Init();

    optical_uart6_dma_running = 0U;
    optical_uart6_dma_pos = 0U;
    optical_uart6_dma_start_error_count = 0U;
    optical_uart6_dma_restart_count = 0U;
    optical_uart6_error_count = 0U;
    optical_uart6_last_error_code = 0U;
    optical_uart6_byte_count = 0U;
    optical_uart6_last_byte = 0U;
    optical_uart6_recent_index = 0U;
    optical_uart6_pending_bytes = 0U;
    optical_uart6_max_pending_bytes = 0U;
    optical_uart6_processed_bytes_last = 0U;
    optical_uart6_process_limit_count = 0U;
    for (uint8_t i = 0U; i < OPTICAL_DIRECT_RECENT_SIZE; i++) {
        optical_uart6_recent_bytes[i] = 0U;
    }
    optical_uart6_rx_pin_level = 1U;
    optical_uart6_rx_pin_edge_count = 0U;
    optical_uart6_rx_pin_low_count = 0U;
    s_rx_pin_sample_div = OPTICAL_DIRECT_RX_PIN_SAMPLE_DIV - 1U;
    optical_uart6_msp_frame_count = 0U;
    optical_uart6_msp_crc_error_count = 0U;
    optical_uart6_micolink_frame_count = 0U;
    optical_uart6_micolink_checksum_error_count = 0U;
    optical_uart6_unknown_frame_count = 0U;

    optical_direct_reset_micolink();
    optical_direct_reset_msp();
    s_direct_seq = 0U;
    s_msp_distance_mm = 0;
    s_msp_distance_valid = 0U;
    s_msp_range_quality = 0U;
    s_msp_range_quality_valid = 0U;
    s_msp_flow_x_raw = 0;
    s_msp_flow_y_raw = 0;
    s_msp_flow_valid = 0U;
    s_msp_flow_quality = 0U;
    s_msp_flow_quality_valid = 0U;
    s_msp_seen_flow_frame = 0U;

    optical_direct_start_rx_dma();
}

void OpticalDirect_Process(void)
{
    uint16_t pos;
    uint16_t pending;
    uint16_t to_process;
    uint16_t processed = 0U;

    s_rx_pin_sample_div++;
    if (s_rx_pin_sample_div >= OPTICAL_DIRECT_RX_PIN_SAMPLE_DIV) {
        s_rx_pin_sample_div = 0U;
        optical_direct_sample_rx_pin();
    }

    if (huart6.hdmarx == NULL) {
        optical_uart6_dma_running = 0U;
        optical_uart6_pending_bytes = 0U;
        optical_uart6_processed_bytes_last = 0U;
        return;
    }

    pos = (uint16_t)(OPTICAL_DIRECT_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    if (pos >= OPTICAL_DIRECT_DMA_SIZE) {
        pos = 0U;
    }
    optical_uart6_dma_pos = pos;
    optical_uart6_dma_running = 1U;

    pending = optical_direct_pending_from_pos(pos);
    optical_uart6_pending_bytes = pending;
    if (pending > optical_uart6_max_pending_bytes) {
        optical_uart6_max_pending_bytes = pending;
    }
    if (pending == 0U) {
        optical_uart6_processed_bytes_last = 0U;
        return;
    }

    to_process = pending;
    if (to_process > OPTICAL_DIRECT_MAX_BYTES_PER_CALL) {
        to_process = OPTICAL_DIRECT_MAX_BYTES_PER_CALL;
        optical_uart6_process_limit_count++;
    }

    while (processed < to_process) {
        optical_direct_record_byte(s_uart6_dma_buf[s_uart6_dma_old_pos]);
        s_uart6_dma_old_pos++;
        if (s_uart6_dma_old_pos >= OPTICAL_DIRECT_DMA_SIZE) {
            s_uart6_dma_old_pos = 0U;
        }
        processed++;
    }

    optical_uart6_processed_bytes_last = processed;
    optical_uart6_pending_bytes = optical_direct_pending_from_pos(pos);
}

void OpticalDirect_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        volatile uint32_t temp;

        optical_uart6_error_count++;
        optical_uart6_last_error_code = huart->ErrorCode;
        HAL_UART_DMAStop(&huart6);
        temp = huart->Instance->SR;
        temp = huart->Instance->DR;
        (void)temp;
        optical_uart6_dma_restart_count++;
        optical_direct_reset_micolink();
        optical_direct_reset_msp();
        optical_direct_start_rx_dma();
    }
}
