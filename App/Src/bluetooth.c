/*
 * Bluetooth 命令集（通过 huart3 接收）
 * 格式：TEXT 行命令，结尾由蓝牙模块发送空闲中断（通常以 CR/LF 或 \n）触发
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
 *
 * 注意事项：
 * - 本实现使用 strtof、snprintf 等函数，可能增加库体积（在使用 newlib-nano 时注意浮点支持的链接选项）。
 * - 对关键 PID 参数做了上下限检查，且更新时会短暂禁用中断以保证原子性，但仍建议在台架上小幅度逐步调整参数。
 * - 若需要保存配置到 Flash，请实现 SAVE/LOAD 命令并限制写入频率以保护 Flash 寿命。
 */

#include "bluetooth.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>

// single-byte interrupt receive implementation (more robust)
static char line_buf[128];
static uint16_t line_idx = 0;
static uint8_t rx_byte;

void bluetooth_init(void) {
    // start single-byte interrupt reception on huart3 (Bluetooth)
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

// HAL UART receive complete callback (called on each received byte)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3){
        char c = (char)rx_byte;
        // append to line buffer if space
        if(line_idx < sizeof(line_buf)-1){
            line_buf[line_idx++] = c;
            // if end of line detected, handle command
            if(c == '\n'){
                line_buf[line_idx] = '\0';
                // trim possible leading/trailing whitespace will be done by handler
                char resp[256] = {0};
                int ok = pid_handle_command(line_buf, resp, sizeof(resp));
                if(resp[0] != '\0'){
                    HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)strlen(resp), 200);
                } else {
                    if(ok){ const char *okmsg = "OK\r\n"; HAL_UART_Transmit(&huart3, (uint8_t*)okmsg, (uint16_t)strlen(okmsg), 200); }
                    else { const char *err = "ERR\r\n"; HAL_UART_Transmit(&huart3, (uint8_t*)err, (uint16_t)strlen(err), 200); }
                }
                // reset index for next line
                line_idx = 0;
            }
        } else {
            // overflow: reset buffer
            line_idx = 0;
        }
        // restart reception for next byte
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
