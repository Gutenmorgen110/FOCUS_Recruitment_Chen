/* 编码器测速，PCNT 硬件正交解码。
 * 逻辑改自 fishbot_motion_control 的 Esp32PcntEncoder，但从 legacy 的
 * driver/pcnt.h 换成了 IDF 5.x 的 driver/pulse_cnt.h：
 *   建单元/通道  pcnt_unit_config()      -> pcnt_new_unit() + pcnt_new_channel()
 *   边沿动作     pos_mode/neg_mode       -> pcnt_channel_set_edge_action()
 *   电平动作     hctrl_mode/lctrl_mode   -> pcnt_channel_set_level_action()
 *   溢出累加     中断里手工 += ±100       -> flags.accum_count = 1 交给驱动
 *   毛刺滤波     pcnt_set_filter_value() -> pcnt_unit_set_glitch_filter()
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "encoder.h"

static const char *TAG = "encoder";

/* 避开电机占的 4~7 */
#define ENC_GPIO_A      GPIO_NUM_15
#define ENC_GPIO_B      GPIO_NUM_16

#define ENC_HIGH_LIMIT  100
#define ENC_LOW_LIMIT  (-100)

/* 滤掉比这还窄的脉冲 */
#define ENC_GLITCH_NS   12500

#define ENC_SAMPLE_MS   50
#define ENCODER_PPR     44      /* 11 线 × 4 倍频；减速电机要再乘减速比，按实物标定 */

static pcnt_unit_handle_t    s_unit;
static pcnt_channel_handle_t s_chan_a;
static pcnt_channel_handle_t s_chan_b;
static volatile float        s_rpm;
static volatile bool         s_ready;

static void rpm_task(void *arg)
{
    int last = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENC_SAMPLE_MS));

        int now = 0;
        if (pcnt_unit_get_count(s_unit, &now) != ESP_OK) {
            continue;
        }
        int delta = now - last;
        last = now;

        s_rpm = (float)delta * 60000.0f / ((float)ENCODER_PPR * ENC_SAMPLE_MS);
    }
}

esp_err_t encoder_init(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit  = ENC_LOW_LIMIT,
        .high_limit = ENC_HIGH_LIMIT,
        .flags.accum_count = 1,     /* 溢出由驱动累加，get_count 直接给总脉冲数 */
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &s_unit), TAG, "建 PCNT 单元失败");

    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = ENC_GLITCH_NS };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s_unit, &filt), TAG, "设滤波失败");

    /* 通道 A：边沿取 A 相，电平看 B 相决定加还是减 */
    pcnt_chan_config_t ca = {
        .edge_gpio_num  = ENC_GPIO_A,
        .level_gpio_num = ENC_GPIO_B,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &ca, &s_chan_a), TAG, "建通道 A 失败");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(
        s_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                  PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "A 边沿动作失败");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(
        s_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "A 电平动作失败");

    /* 通道 B：反过来，边沿取 B 相、电平看 A 相 */
    pcnt_chan_config_t cb = {
        .edge_gpio_num  = ENC_GPIO_B,
        .level_gpio_num = ENC_GPIO_A,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_unit, &cb, &s_chan_b), TAG, "建通道 B 失败");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(
        s_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                  PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "B 边沿动作失败");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(
        s_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "B 电平动作失败");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_unit), TAG, "使能失败");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_unit), TAG, "清计数失败");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_unit), TAG, "启动失败");

    if (xTaskCreate(rpm_task, "encoder_rpm", 3072, NULL, 9, NULL) != pdPASS) {
        ESP_LOGE(TAG, "测速任务创建失败");
        return ESP_FAIL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "PCNT 就绪 A=%d B=%d，每转 %d 脉冲，%dms 采样",
             ENC_GPIO_A, ENC_GPIO_B, ENCODER_PPR, ENC_SAMPLE_MS);
    return ESP_OK;
}

float encoder_get_rpm(void)
{
    return s_rpm;
}

bool encoder_is_ready(void)
{
    return s_ready;
}
