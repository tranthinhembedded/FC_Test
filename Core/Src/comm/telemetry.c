#include "comm/telemetry.h"

#include "platform/usart.h"
#include "sensor/sensor_common.h"

#include <stdio.h>

void Send_Telemetry(void)
{
    static char tx_buf[128];
    int len;

    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf), "{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"v\":%.1f}\n",
        Complimentary_Filter.Euler_Angle_Deg[0],
        Complimentary_Filter.Euler_Angle_Deg[1],
        Complimentary_Filter.Euler_Angle_Deg[2],
        vbat);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)len, 10);
    }
}
