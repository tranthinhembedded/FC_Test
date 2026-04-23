#ifndef CORE_APP_COMMUNICATION_PIDTUNING_PID_TUNING_H_
#define CORE_APP_COMMUNICATION_PIDTUNING_PID_TUNING_H_

#include "platform/usart.h"

void PidTuning_Init(void);
void PidTuning_ProcessPendingCommand(void);
void PidTuning_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size);
void PidTuning_HandleUartError(UART_HandleTypeDef *huart);

#endif /* CORE_APP_COMMUNICATION_PIDTUNING_PID_TUNING_H_ */
