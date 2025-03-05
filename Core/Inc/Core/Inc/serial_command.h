#ifndef __SERIAL_COMMAND_H
#define __SERIAL_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

void serial_command(UART_HandleTypeDef *huart, uint8_t *direction, uint16_t *speed);

#ifdef __cplusplus
}
#endif

#endif // __SERIAL_COMMAND_H