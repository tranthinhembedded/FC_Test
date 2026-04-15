#include "sensor/gps.h"
#include "platform/usart.h"
#include <string.h>
#include <stdlib.h>

GPS_Data_t GPS_Data;

#define GPS_DMA_BUFFER_SIZE 256
static uint8_t gps_dma_buf[GPS_DMA_BUFFER_SIZE];
static uint16_t gps_old_pos = 0;

static char gps_line_buf[128];
static uint8_t gps_line_idx = 0;

void GPS_Init(void)
{
    memset(&GPS_Data, 0, sizeof(GPS_Data));
    gps_old_pos = 0;
    
    /* Start circular DMA receiving for USART2 */
    HAL_UART_Receive_DMA(&huart2, gps_dma_buf, GPS_DMA_BUFFER_SIZE);
}

static void Parse_NMEA_Sentence(char *sentence)
{
    /* Supports parsing both GPS (GPGGA) and multi-GNSS (GNGGA) */
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char *ptr = sentence;
        int comma_count = 0;
        
        while (*ptr) {
            if (*ptr == ',') {
                comma_count++;
                if (comma_count == 2) {
                    /* Latitude parsing: DDMM.MMMM -> Decimal Degrees */
                    float raw_lat = atof(ptr + 1);
                    int degrees = (int)(raw_lat / 100);
                    float minutes = raw_lat - (degrees * 100.0f);
                    GPS_Data.latitude = degrees + (minutes / 60.0f);
                } else if (comma_count == 3) {
                    /* South */
                    if (*(ptr + 1) == 'S') GPS_Data.latitude = -GPS_Data.latitude;
                } else if (comma_count == 4) {
                    /* Longitude parsing: DDDMM.MMMM -> Decimal Degrees */
                    float raw_lon = atof(ptr + 1);
                    int degrees = (int)(raw_lon / 100);
                    float minutes = raw_lon - (degrees * 100.0f);
                    GPS_Data.longitude = degrees + (minutes / 60.0f);
                } else if (comma_count == 5) {
                    /* West */
                    if (*(ptr + 1) == 'W') GPS_Data.longitude = -GPS_Data.longitude;
                } else if (comma_count == 6) {
                    /* Fix quality */
                    GPS_Data.fix_quality = atoi(ptr + 1);
                    GPS_Data.is_valid = (GPS_Data.fix_quality > 0);
                } else if (comma_count == 7) {
                    /* Satellites */
                    GPS_Data.satellites = atoi(ptr + 1);
                } else if (comma_count == 9) {
                    /* Altitude */
                    GPS_Data.altitude = atof(ptr + 1);
                }
            }
            ptr++;
        }
    }
}

static void Process_GPS_Byte(uint8_t byte)
{
    if (byte == '$') {
        gps_line_idx = 0;
        gps_line_buf[gps_line_idx++] = byte;
    } else if (gps_line_idx > 0 && gps_line_idx < sizeof(gps_line_buf) - 1) {
        if (byte == '\r' || byte == '\n') {
            gps_line_buf[gps_line_idx] = '\0';
            Parse_NMEA_Sentence(gps_line_buf);
            gps_line_idx = 0;
        } else {
            gps_line_buf[gps_line_idx++] = byte;
        }
    }
}

void GPS_Process_DMA_Ring_Buffer(void)
{
    uint16_t pos = GPS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    
    if (pos != gps_old_pos) {
        if (pos > gps_old_pos) {
            for (uint16_t i = gps_old_pos; i < pos; i++) {
                Process_GPS_Byte(gps_dma_buf[i]);
            }
        } else {
            for (uint16_t i = gps_old_pos; i < GPS_DMA_BUFFER_SIZE; i++) {
                Process_GPS_Byte(gps_dma_buf[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                Process_GPS_Byte(gps_dma_buf[i]);
            }
        }
        gps_old_pos = pos;
    }
}
