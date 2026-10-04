/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "led_ctrl.h"
#include "esp_log.h"

static const char *TAG = "led_ctrl";


#define LEDC_LS_TIMER          LEDC_TIMER_0
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_GPIO       GPIO_NUM_8
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES          LEDC_TIMER_13_BIT
#define LEDC_FREQ_HZ           4000
#define LEDC_MAX_DUTY          ((1 << 13) - 1)

static bool s_is_on = false;
static int  s_brightness = 100;

static void update_hw(void)
{
    uint32_t duty = s_is_on ? (uint32_t)s_brightness * LEDC_MAX_DUTY / 100 : 0;
    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL, duty);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH0_CHANNEL);
}

/* ==================== 初始化 ==================== */
void led_ctrl_init(void)
{
    /* 1. 配置 LEDC 定时器 */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .speed_mode      = LEDC_LS_MODE,
        .timer_num       = LEDC_LS_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* 2. 配置 LEDC 通道 */
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_LS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_LS_TIMER,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    /* 3. 初始状态：灭 */
    s_is_on = false;
    s_brightness = 100;
    update_hw();

    ESP_LOGI(TAG, "LEDC initialized on GPIO %d, freq=%dHz",
             LEDC_LS_CH0_GPIO, LEDC_FREQ_HZ);
}

/* ==================== 对外接口 ==================== */
void led_ctrl_on(void)
{
    s_is_on = true;
    update_hw();
    ESP_LOGI(TAG, "LED ON, brightness=%d%%", s_brightness);
}

void led_ctrl_off(void)
{
    s_is_on = false;
    update_hw();
    ESP_LOGI(TAG, "LED OFF");
}

void led_ctrl_set_brightness(int percent)
{
    if (percent < 0 || percent > 100) {
        ESP_LOGW(TAG, "brightness %d out of range", percent);
        return;
    }
    s_is_on = true;
    s_brightness = percent;
    update_hw();
    ESP_LOGI(TAG, "LED brightness=%d%%", percent);
}