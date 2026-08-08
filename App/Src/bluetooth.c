#include "bluetooth.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
char rx_buffer[10];

void bluetooth_init(void) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)rx_buffer, sizeof(rx_buffer));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        // Process the received data in rx_buffer
        // For example, you can print it or handle it as needed
        // After processing, you can restart the reception
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)rx_buffer, sizeof(rx_buffer));
    }
}