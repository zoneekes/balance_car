#include "bluetooth.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
char rx_buffer[10];

void bluetooth_init(void) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)rx_buffer, sizeof(rx_buffer));
}