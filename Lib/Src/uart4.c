#include "uart4.h"
#include "wit_c_sdk.h"
#include <stdio.h>

extern UART_HandleTypeDef huart6;

void Usart4Init(unsigned int uiBaud)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = uiBaud;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        __disable_irq();
        while (1)
        {
        }
    }
}


void Uart4Send(unsigned char *p_data, unsigned int uiSize)
{
    HAL_UART_Transmit(&huart6, p_data, uiSize, 100);
}

