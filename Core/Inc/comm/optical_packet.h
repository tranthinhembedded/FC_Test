#ifndef OPTICAL_PACKET_H
#define OPTICAL_PACKET_H

#include <stdint.h>
#include <string.h>

#define OPTICAL_SYNC1 0xAAu
#define OPTICAL_SYNC2 0x55u
#define OPTICAL_VERSION 1u
#define OPTICAL_TYPE_FLOW 0x10u
#define OPTICAL_PAYLOAD_LEN 20u
#define OPTICAL_FRAME_LEN 27u

#define OPTICAL_FLAG_DISTANCE_VALID       (1u << 0)
#define OPTICAL_FLAG_FLOW_VALID           (1u << 1)
#define OPTICAL_FLAG_RANGE_QUALITY_VALID  (1u << 2)
#define OPTICAL_FLAG_FLOW_QUALITY_VALID   (1u << 3)
#define OPTICAL_FLAG_DISTANCE_OUT_RANGE   (1u << 4)
#define OPTICAL_FLAG_PROTOCOL_MSP         (1u << 8)
#define OPTICAL_FLAG_PROTOCOL_MICOLINK    (1u << 9)

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t flags;
    int32_t distance_mm;
    int32_t flow_x_raw;
    int32_t flow_y_raw;
    uint8_t range_quality;
    uint8_t flow_quality;
} optical_payload_t;

typedef struct {
    uint8_t seq;
    optical_payload_t payload;
} optical_packet_t;

typedef struct {
    uint8_t index;
    uint8_t buf[OPTICAL_FRAME_LEN];
} optical_parser_t;

static inline void optical_parser_init(optical_parser_t *parser)
{
    parser->index = 0;
}

static inline uint8_t optical_crc8_dvb_s2_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80u) {
            crc = (uint8_t)((crc << 1) ^ 0xD5u);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint8_t optical_crc8_dvb_s2(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = optical_crc8_dvb_s2_update(crc, data[i]);
    }
    return crc;
}

static inline int optical_packet_feed(optical_parser_t *parser, uint8_t byte, optical_packet_t *out)
{
    if (parser->index == 0) {
        if (byte != OPTICAL_SYNC1) {
            return 0;
        }
        parser->buf[parser->index++] = byte;
        return 0;
    }

    if (parser->index == 1) {
        if (byte != OPTICAL_SYNC2) {
            parser->index = (byte == OPTICAL_SYNC1) ? 1u : 0u;
            parser->buf[0] = OPTICAL_SYNC1;
            return 0;
        }
        parser->buf[parser->index++] = byte;
        return 0;
    }

    parser->buf[parser->index++] = byte;
    if (parser->index < OPTICAL_FRAME_LEN) {
        return 0;
    }

    parser->index = 0;
    if (parser->buf[2] != OPTICAL_VERSION ||
        parser->buf[3] != OPTICAL_TYPE_FLOW ||
        parser->buf[4] != OPTICAL_PAYLOAD_LEN) {
        return 0;
    }

    uint8_t crc = optical_crc8_dvb_s2(&parser->buf[2], OPTICAL_FRAME_LEN - 3u);
    if (crc != parser->buf[OPTICAL_FRAME_LEN - 1u]) {
        return 0;
    }

    out->seq = parser->buf[5];
    memcpy(&out->payload, &parser->buf[6], sizeof(out->payload));
    return 1;
}

#endif
