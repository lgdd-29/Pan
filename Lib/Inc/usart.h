#ifndef __USART_H
#define __USART_H

#include "main.h"
#include <stdbool.h>

void usart_SendCmd(UART_HandleTypeDef *huart, uint8_t *cmd, uint8_t len);
#endif
