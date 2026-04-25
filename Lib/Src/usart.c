#include "usart.h"


// 引用CubeMX定义的串口句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/**
 * @brief  串口发送多个字节
 */
void usart_SendCmd(UART_HandleTypeDef *huart, uint8_t *cmd, uint8_t len)
{
    // 直接一次性发送整个帧，避免单字节循环发送造成的调用开销和可能的字节间隙
    HAL_UART_Transmit(huart, cmd, len, 50);
}
