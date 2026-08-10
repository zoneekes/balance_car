/*
 * Bluetooth 命令集（通过 huart3 接收）
* 格式：文本行命令，通常以 CR/LF 结束（\r\n）。
 *
 * 设置参数（KEY=VALUE）示例：
 *   V_KP=200.0    -- 设置直立环 Kp（允许范围：0.0 .. 1000.0）
 *   V_KD=2.04     -- 设置直立环 Kd（允许范围：0.0 .. 10.0）
 *   S_KP=0.6      -- 设置速度环 Kp（允许范围：0.0 .. 5.0）
 *   S_KI=0.003    -- 设置速度环 Ki（允许范围：0.0 .. 1.0）
 *   ST_KP=1.0     -- 设置转向环 Kp（允许范围：0.0 .. 20.0）
 *   ST_KD=0.1     -- 设置转向环 Kd（允许范围：0.0 .. 5.0）
 *   T_SPEED=100   -- 设置目标速度（整数，限制 -2000 .. 2000）
 *   T_ANGLE=5     -- 设置目标角度（整数，限制 -30 .. 30）
 *
 * 查询参数：
 *   GET ALL       -- 返回所有当前参数值（单行字符串）
 *
 * 运动控制（CMD）：
 *   CMD=FORWARD[:pwm]   -- 前进，pwm 为可选整数（默认1000）
 *   CMD=BACKWARD[:pwm]  -- 后退
 *   CMD=LEFT[:pwm]      -- 向左原地转
 *   CMD=RIGHT[:pwm]     -- 向右原地转
 *   CMD=STOP            -- 停止电机（PWM=0）
 *
 * 响应格式（通过蓝牙回传）：
 *   成功：OK KEY=VALUE\r\n 或 OK CMD=FORWARD PWM=1000\r\n
 *   查询返回：V_KP=200.000 V_KD=2.040 ... T_ANGLE=0\r\n
 *   错误：ERR reason\r\n
*/

#include "bluetooth.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>

#define BT_LINE_BUF_SIZE 128
#define BT_TX_QUEUE_SIZE 4
#define BT_TX_MAX_LEN 256

typedef struct {
   uint16_t len;
   char data[BT_TX_MAX_LEN];
} bt_tx_msg_t;

static char line_buf[BT_LINE_BUF_SIZE];
static volatile uint16_t line_idx = 0;
static uint8_t rx_byte;
static bt_tx_msg_t tx_queue[BT_TX_QUEUE_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile uint8_t tx_count = 0;
static volatile uint8_t tx_busy = 0;

static void bluetooth_enqueue_tx(const char *data, uint16_t len)
{
   if (data == NULL || len == 0) {
       return;
   }
   if (len > BT_TX_MAX_LEN - 1) {
       len = BT_TX_MAX_LEN - 1;
   }

   __disable_irq();
   uint8_t next_tail = (tx_tail + 1u) % BT_TX_QUEUE_SIZE;
   if (next_tail == tx_head) {
       __enable_irq();
       return;
   }
   memcpy(tx_queue[tx_tail].data, data, len);
   tx_queue[tx_tail].data[len] = '\0';
   tx_queue[tx_tail].len = len;
   tx_tail = next_tail;
   tx_count++;
   __enable_irq();
}

static void bluetooth_start_tx(void)
{
   if (tx_busy || tx_count == 0) {
       return;
   }

   tx_busy = 1;
   HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx_queue[tx_head].data, tx_queue[tx_head].len);
}

void bluetooth_init(void)
{
   line_idx = 0;
   tx_head = 0;
   tx_tail = 0;
   tx_count = 0;
   tx_busy = 0;
   HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void bluetooth_poll(void)
{
   if (!tx_busy && tx_count > 0) {
       bluetooth_start_tx();
   }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART3) {
       char c = (char)rx_byte;
       if (c == '\r' || c == '\n') {
           if (line_idx > 0) {
               line_buf[line_idx] = '\0';
               char resp[256] = {0};
               int ok = pid_handle_command(line_buf, resp, sizeof(resp));
               if (resp[0] != '\0') {
                   bluetooth_enqueue_tx(resp, (uint16_t)strlen(resp));
               } else {
                   if (ok) {
                       bluetooth_enqueue_tx("OK\r\n", 4);
                   } else {
                       bluetooth_enqueue_tx("ERR\r\n", 5);
                   }
               }
           }
           line_idx = 0;
       } else {
           if (line_idx < BT_LINE_BUF_SIZE - 1) {
               line_buf[line_idx++] = c;
           } else {
               line_buf[BT_LINE_BUF_SIZE - 1] = '\0';
               bluetooth_enqueue_tx("ERR OVF\r\n", 9);
               line_idx = 0;
           }
       }

       HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
   }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART3) {
       if (tx_count > 0) {
           tx_head = (tx_head + 1u) % BT_TX_QUEUE_SIZE;
           tx_count--;
       }
       tx_busy = 0;
       bluetooth_poll();
   }
}
