#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

esp_err_t app_ctrl_init(void);

/* 执行一条指令，应答写进 resp；resp[0]=='\0' 表示这条不用回 */
void app_ctrl_exec(const char *cmd, char *resp, size_t len);

/* 串口收到整行的回调，UART0 与 UART1（蓝牙）共用 */
void app_ctrl_on_line(uart_port_t port, const char *line);

#ifdef __cplusplus
}
#endif
