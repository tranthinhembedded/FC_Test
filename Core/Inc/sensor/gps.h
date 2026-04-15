#ifndef CORE_INC_SENSOR_GPS_H_
#define CORE_INC_SENSOR_GPS_H_

#include <stdint.h>

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint8_t fix_quality;
    uint8_t satellites;
    uint8_t is_valid;
} GPS_Data_t;

extern GPS_Data_t GPS_Data;

void GPS_Init(void);
void GPS_Process_DMA_Ring_Buffer(void);

#endif /* CORE_INC_SENSOR_GPS_H_ */
