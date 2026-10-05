/* 串口命令行前端：中断收数进环形缓冲，事件任务按行切分，不轮询不阻塞。
 * UART0 / UART1 共用同一套解析。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "uart_cmd.h"

static const char *TAG = "uart_cmd";

#define UART_BUF_SIZE   1024
#define LINE_BUF_SIZE   128

// 断行策略待定：也可改成带超时的 xQueueReceive，超时即把缓冲区当一整行交出去
// #define LINE_IDLE_MS    50

typedef struct {
    QueueHandle_t queue;
    uart_line_cb_t cb;
} uart_ctx_t;

static uart_ctx_t s_ctx[UART_NUM_MAX];

void uart_cmd_reply(uart_port_t port, const char *msg)
{
    uart_write_bytes(port, msg, strlen(msg));
    uart_write_bytes(port, "\r\n", 2);
}

static void uart_event_task(void *arg)
{
    uart_port_t port = (uart_port_t)(intptr_t)arg;
    uart_ctx_t *ctx = &s_ctx[port];

    uart_event_t event;
    uint8_t *buf = malloc(UART_BUF_SIZE);
    char *line = malloc(LINE_BUF_SIZE);
    int idx = 0;                /* 跨事件保留，指令可能被切成几段到达 */

    for (;;) {
        if (xQueueReceive(ctx->queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type != UART_DATA) {
            /* 缓冲溢出之类的异常，冲掉重来 */
            if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
                ESP_LOGW(TAG, "uart%d event %d，清空缓冲", port, event.type);
                uart_flush_input(port);
                xQueueReset(ctx->queue);
                idx = 0;
            }
            continue;
        }

        int len = uart_read_bytes(port, buf,
                                  event.size < UART_BUF_SIZE ? event.size : UART_BUF_SIZE,
                                  pdMS_TO_TICKS(10));

        /* 调试用：打印收到的原始字节 */
        if (len > 0) {
            char hex[3 * 40 + 1];
            int k = 0;
            for (int i = 0; i < len && i < 40 && k < (int)sizeof(hex) - 4; i++) {
                k += snprintf(hex + k, sizeof(hex) - k, "%02X ", (unsigned char)buf[i]);
            }
            ESP_LOGI(TAG, "uart%d 收到 %d 字节: %s", port, len, hex);
        }

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (idx > 0) {          /* 收满一行才执行，空行忽略 */
                    line[idx] = '\0';
                    if (ctx->cb) {
                        ctx->cb(port, line);
                    }
                    idx = 0;
                }
            } else if (idx < LINE_BUF_SIZE - 1) {
                line[idx++] = c;
            } else {
                idx = 0;                /* 太长，丢整条 */
                uart_cmd_reply(port, "ERROR");
            }
        }
    }
}

esp_err_t uart_cmd_init(uart_port_t port, int tx_pin, int rx_pin, int baud, uart_line_cb_t cb)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    s_ctx[port].cb = cb;

    ESP_ERROR_CHECK(uart_driver_install(port, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2,
                                        20, &s_ctx[port].queue, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char name[16];
    snprintf(name, sizeof(name), "uart%d_task", port);
    if (xTaskCreate(uart_event_task, name, 4096, (void *)(intptr_t)port, 12, NULL) != pdPASS) {
        ESP_LOGE(TAG, "uart%d 任务创建失败", port);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "uart%d 就绪 TX=%d RX=%d %d 8N1", port, tx_pin, rx_pin, baud);
    return ESP_OK;
}
