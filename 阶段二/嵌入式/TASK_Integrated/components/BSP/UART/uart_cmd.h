#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/uart.h"

/* 收到一整行时回调，应答由回调自己发 */
typedef void (*uart_line_cb_t)(uart_port_t port, const char *line);

esp_err_t uart_cmd_init(uart_port_t port, int tx_pin, int rx_pin, int baud, uart_line_cb_t cb);

void uart_cmd_reply(uart_port_t port, const char *msg);

#ifdef __cplusplus
}
#endif
