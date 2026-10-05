/* 编码器测速，PCNT 硬件正交解码 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "encoder.h"

static const char *TAG = "encoder";

/* 避开电机占的 4~7 */
#define ENC_GPIO_A      GPIO_NUM_15
#define ENC_GPIO_B      GPIO_NUM_16

/* 计数上下限，到限由驱动自动累加到总数（accum_count） */
#define ENC_HIGH_LIMIT  100
#define ENC_LOW_LIMIT  (-100)

/* 滤掉比这还窄的脉冲 */
#define ENC_GLITCH_NS   12500

#define ENC_SAMPLE_MS   50
#define ENCODER_PPR     44      /* 11 线 × 4 倍频；减速电机要再乘减速比 */

static pcnt_unit_handle_t s_unit;
static volatile float s_rpm;

static void rpm_task(void *arg)
{
    int last = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENC_SAMPLE_MS));

        int now = 0;
        pcnt_unit_get_count(s_unit, &now);
        s_rpm = (float)(now - last) * 60000.0f / ((float)ENCODER_PPR * ENC_SAMPLE_MS);
        last = now;
    }
}

void encoder_init(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit  = ENC_LOW_LIMIT,
        .high_limit = ENC_HIGH_LIMIT,
        .flags.accum_count = 1,
    };
    pcnt_new_unit(&ucfg, &s_unit);

    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = ENC_GLITCH_NS };
    pcnt_unit_set_glitch_filter(s_unit, &filt);

    /* 通道 A：边沿取 A 相，电平看 B 相决定加还是减 */
    pcnt_chan_config_t ca = { .edge_gpio_num = ENC_GPIO_A, .level_gpio_num = ENC_GPIO_B };
    pcnt_channel_handle_t ch_a;
    pcnt_new_channel(s_unit, &ca, &ch_a);
    pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* 通道 B：反过来 */
    pcnt_chan_config_t cb = { .edge_gpio_num = ENC_GPIO_B, .level_gpio_num = ENC_GPIO_A };
    pcnt_channel_handle_t ch_b;
    pcnt_new_channel(s_unit, &cb, &ch_b);
    pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(s_unit);
    pcnt_unit_clear_count(s_unit);
    pcnt_unit_start(s_unit);

    xTaskCreate(rpm_task, "encoder_rpm", 3072, NULL, 9, NULL);
    ESP_LOGI(TAG, "PCNT 就绪 A=%d B=%d，每转 %d 脉冲", ENC_GPIO_A, ENC_GPIO_B, ENCODER_PPR);
}

float encoder_get_rpm(void)
{
    return s_rpm;
}
