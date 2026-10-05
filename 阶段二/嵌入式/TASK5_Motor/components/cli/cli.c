/* 串口命令行：沿用 TASK3思路
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "cli.h"
#include "motor_ctrl.h"

static const char *TAG = "cli";

#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define UART_BUF_SIZE   1024
#define LINE_BUF_SIZE   128

static QueueHandle_t s_uart_queue;

static void reply(const char *msg)
{
    uart_write_bytes(UART_PORT, msg, strlen(msg));
    uart_write_bytes(UART_PORT, "\r\n", 2);
}

/* 解析 0~100 的整数 */
static bool parse_percent(const char *s, int *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    int val = 0;
    for (const char *p = s; *p; p++) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        val = val * 10 + (*p - '0');
        if (val > 100) {
            return false;       
        }
    }
    *out = val;
    return true;
}

void cli_process_cmd(char *cmd)
{
    char *tok = strtok(cmd, " \t");
    if (tok == NULL) {
        return;                 
    }

    if (strcmp(tok, "ON") == 0) {
        if (strtok(NULL, " \t") != NULL) {
            reply("ERROR");
            return;
        }
        motor_ctrl_resume();
        reply("OK");
        return;
    }

    if (strcmp(tok, "OFF") == 0) {
        if (strtok(NULL, " \t") != NULL) {
            reply("ERROR");
            return;
        }
        motor_ctrl_stop();
        reply("OK");
        return;
    }

    if (strcmp(tok, "AUTO") == 0) {
        if (strtok(NULL, " \t") != NULL) {
            reply("ERROR");
            return;
        }
        motor_ctrl_set_sweep();
        reply("OK");
        return;
    }

    if (strcmp(tok, "SPEED") == 0) {
        char *arg = strtok(NULL, " \t");
        int pct = 0;
        //异常情况处理
        if (!parse_percent(arg, &pct) || strtok(NULL, " \t") != NULL) {
            reply("ERROR");
            return;
        }
        motor_ctrl_set_manual(pct);
        reply("OK");
        return;
    }

    if (strcmp(tok, "DIR") == 0) {
        char *arg = strtok(NULL, " \t");
        if (arg == NULL || strtok(NULL, " \t") != NULL) {
            reply("ERROR");
            return;
        }
        if (strcmp(arg, "F") == 0) {
            motor_ctrl_set_dir(true);
        } else if (strcmp(arg, "R") == 0) {
            motor_ctrl_set_dir(false);
        } else {
            reply("ERROR");
            return;
        }
        reply("OK");
        return;
    }

    if (strcmp(tok, "STATUS") == 0) {
        static const char *names[] = {"STOP", "SWEEP", "MANUAL"};
        char buf[64];
        snprintf(buf, sizeof(buf), "MODE=%s TARGET=%d%%",
                 names[motor_ctrl_get_mode()], motor_ctrl_get_target());
        reply(buf);
        return;
    }

    reply("ERROR");
}

static void uart_event_task(void *arg)
{
    uart_event_t event;
    uint8_t *buf = malloc(UART_BUF_SIZE);
    char *line = malloc(LINE_BUF_SIZE);
    int idx = 0;                /* 跨事件保留，指令可能被切成几段到达 */

    for (;;) {
        if (xQueueReceive(s_uart_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type != UART_DATA) {
           
            if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
                ESP_LOGW(TAG, "uart event %d，清空缓冲", event.type);
                uart_flush_input(UART_PORT);
                xQueueReset(s_uart_queue);
                idx = 0;
            }
            continue;
        }

        int len = uart_read_bytes(UART_PORT, buf,event.size < UART_BUF_SIZE ? event.size : UART_BUF_SIZE,pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (idx > 0) {          /* 收满一行才执行，空行忽略 */
                    line[idx] = '\0';
                    cli_process_cmd(line);
                    idx = 0;
                }
            } else if (idx < LINE_BUF_SIZE - 1) {
                line[idx++] = c;
            } else {
                idx = 0;                /* 太长，丢整条 */
                reply("ERROR");
            }
        }
    }
}

esp_err_t cli_start(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2,
    UART_BUF_SIZE * 2,20, &s_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, GPIO_NUM_43, GPIO_NUM_44,
    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    BaseType_t ok = xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "串口任务创建失败");
        return ESP_FAIL;
    }

    reply("READY");
    ESP_LOGI(TAG, "串口命令已就绪，输入 STATUS 查看当前状态");
    return ESP_OK;
}
