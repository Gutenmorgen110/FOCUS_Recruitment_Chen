/* 串口 / 蓝牙 / 网页的指令的集合。做一个启动综合 */
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "motor.h"
#include "motor_ctrl.h"
#include "encoder.h"
#include "uart_cmd.h"
#include "app_ctrl.h"

static const char *TAG = "ctrl";

static bool s_inited;

/* 只收纯数字 */
static bool parse_int(const char *s, int *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    int v = 0;
    for (const char *p = s; *p; p++) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        v = v * 10 + (*p - '0');
        if (v > 100000) {
            return false;
        }
    }
    *out = v;
    return true;
}

static const char *led_mode_name(void)
{
    switch (led_get_mode()) {
    case LED_MODE_BLINK:  return "BLINK";
    case LED_MODE_BREATH: return "BREATH";
    default:              return "MANUAL";
    }
}

static const char *motor_mode_name(void)
{
    switch (motor_ctrl_get_mode()) {
    case MOTOR_MODE_SWEEP:  return "SWEEP";
    case MOTOR_MODE_MANUAL: return "MANUAL";
    case MOTOR_MODE_CLOSED: return "CLOSED";
    default:                return "STOP";
    }
}

static void do_status(char *resp, size_t len)
{
    char mtarget[16];
    if (motor_ctrl_get_mode() == MOTOR_MODE_CLOSED) {
        snprintf(mtarget, sizeof(mtarget), "%dRPM", motor_ctrl_get_target());
    } else {
        snprintf(mtarget, sizeof(mtarget), "%d%%", motor_ctrl_get_target());
    }

    snprintf(resp, len, "LED=%s/%d%% MOTOR=%s/%s ACT=%.1fRPM",
             led_mode_name(), led_get_brightness(),
             motor_mode_name(), mtarget,
             motor_ctrl_get_rpm());
}

static void do_help(char *resp, size_t len)
{
    snprintf(resp, len,
             "LED ON|OFF|<0-100>|BLINK|BREATH" "\r\n"
             "MOTOR ON|OFF|<0-100>|SWEEP|DIR F|R|RPM <n>" "\r\n"
             "STATUS | HELP     (ON/OFF/SPEED 是 LED 的简写)");
}

void app_ctrl_exec(const char *cmd, char *resp, size_t len)
{
    char buf[128];

    resp[0] = '\0';

    /* 没初始化就直接拒掉 */
    if (!s_inited) {
        ESP_LOGE(TAG, "app_ctrl_init() 没调用，拒绝执行 [%s]", cmd);
        snprintf(resp, len, "ERROR: not initialized");
        return;
    }

    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tok = strtok(buf, " \t");
    if (tok == NULL) {
        return;                     /* 空行，不回话 */
    }

    if (strcmp(tok, "LED") == 0) {
        char *a = strtok(NULL, " \t");
        if (a == NULL) {
            snprintf(resp, len, "ERROR");
        } else if (strcmp(a, "ON") == 0) {
            led_on();
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "OFF") == 0) {
            led_off();
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "BLINK") == 0) {
            led_set_mode(LED_MODE_BLINK);
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "BREATH") == 0) {
            led_set_mode(LED_MODE_BREATH);
            snprintf(resp, len, "OK");
        } else {
            int p;
            if (parse_int(a, &p) && p <= 100) {
                led_set_brightness(p);
                snprintf(resp, len, "OK");
            } else {
                snprintf(resp, len, "ERROR");
            }
        }
        return;
    }

    if (strcmp(tok, "MOTOR") == 0) {
        char *a = strtok(NULL, " \t");
        if (a == NULL) {
            snprintf(resp, len, "ERROR");
        } else if (strcmp(a, "ON") == 0) {
            motor_ctrl_resume();
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "OFF") == 0) {
            motor_ctrl_stop();
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "SWEEP") == 0) {
            motor_ctrl_set_sweep();
            snprintf(resp, len, "OK");
        } else if (strcmp(a, "DIR") == 0) {
            char *d = strtok(NULL, " \t");
            if (d && strcmp(d, "F") == 0) {
                motor_ctrl_set_dir(true);
                snprintf(resp, len, "OK");
            } else if (d && strcmp(d, "R") == 0) {
                motor_ctrl_set_dir(false);
                snprintf(resp, len, "OK");
            } else {
                snprintf(resp, len, "ERROR");
            }
        } else if (strcmp(a, "RPM") == 0) {
            int r;
            if (parse_int(strtok(NULL, " \t"), &r)) {
                motor_ctrl_set_closed(r);
                snprintf(resp, len, "OK");
            } else {
                snprintf(resp, len, "ERROR");
            }
        } else {
            int p;
            if (parse_int(a, &p) && p <= 100) {
                motor_ctrl_set_manual(p);
                snprintf(resp, len, "OK");
            } else {
                snprintf(resp, len, "ERROR");
            }
        }
        return;
    }

    if (strcmp(tok, "STATUS") == 0) {
        do_status(resp, len);
        return;
    }

    if (strcmp(tok, "HELP") == 0) {
        do_help(resp, len);
        return;
    }

    /* 简写，沿用串口点灯那套指令名 */
    if (strcmp(tok, "ON") == 0) {
        led_on();
        snprintf(resp, len, "OK");
        return;
    }
    if (strcmp(tok, "OFF") == 0) {
        led_off();
        snprintf(resp, len, "OK");
        return;
    }
    if (strcmp(tok, "SPEED") == 0) {
        int p;
        if (parse_int(strtok(NULL, " \t"), &p) && p <= 100) {
            led_set_brightness(p);
            snprintf(resp, len, "OK");
        } else {
            snprintf(resp, len, "ERROR");
        }
        return;
    }

    snprintf(resp, len, "ERROR");
}

void app_ctrl_on_line(uart_port_t port, const char *line)
{
    char resp[192];
    app_ctrl_exec(line, resp, sizeof(resp));

    /* 调试用 */
    ESP_LOGI(TAG, "uart%d 收到行 [%s] → 回 [%s]", port, line, resp[0] ? resp : "不回");

    if (resp[0] != '\0') {
        uart_cmd_reply(port, resp);
    }
}

esp_err_t app_ctrl_init(void)
{
    ESP_ERROR_CHECK(led_init());
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(motor_ctrl_start());

    encoder_init();

    s_inited = true;
    ESP_LOGI(TAG, "控制层就绪，HELP 看指令");
    return ESP_OK;
}
