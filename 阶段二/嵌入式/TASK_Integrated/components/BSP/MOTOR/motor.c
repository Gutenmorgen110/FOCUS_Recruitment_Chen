/* TB6612 驱动：AIN1/AIN2 定方向，PWMA 走 LEDC 调速。
 * STBY 必须拉高，否则整片芯片待机、电机完全不动。
 */
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "motor.h"

static const char *TAG = "motor";

/* 避开 strapping(0/3/45/46)、USB(19/20)、UART0(43/44)、PSRAM(33~37) */
#define PIN_PWMA    GPIO_NUM_4
#define PIN_AIN1    GPIO_NUM_5
#define PIN_AIN2    GPIO_NUM_6
#define PIN_STBY    GPIO_NUM_7

/* 20kHz 在人耳听阈之上，听不到啸叫；10bit 是被 20kHz 逼出来的（20k × 2^13 > 80MHz） */
#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_LEDC_TIMER    LEDC_TIMER_1
#define MOTOR_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_CHANNEL  LEDC_CHANNEL_1
#define MOTOR_LEDC_RES      LEDC_TIMER_10_BIT
#define MOTOR_DUTY_MAX      ((1 << 10) - 1)

static bool s_forward = true;
static float s_duty = 0.0f;

static void apply_duty(void)
{
    uint32_t raw = (uint32_t)(s_duty * MOTOR_DUTY_MAX + 0.5f);
    ledc_set_duty_and_update(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL, raw, 0);
}

static void apply_dir(void)
{
    gpio_set_level(PIN_AIN1, s_forward ? 1 : 0);
    gpio_set_level(PIN_AIN2, s_forward ? 0 : 1);
}

esp_err_t motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) | (1ULL << PIN_STBY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* 先按住待机、方向清零，配好 PWM 再放行，免得接上电就猛冲 */
    gpio_set_level(PIN_STBY, 0);
    gpio_set_level(PIN_AIN1, 0);
    gpio_set_level(PIN_AIN2, 0);

    ledc_timer_config_t timer = {
        .duty_resolution = MOTOR_LEDC_RES,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .speed_mode      = MOTOR_LEDC_MODE,
        .timer_num       = MOTOR_LEDC_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .channel    = MOTOR_LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = PIN_PWMA,
        .speed_mode = MOTOR_LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    s_duty = 0.0f;
    s_forward = true;
    apply_dir();
    apply_duty();

    gpio_set_level(PIN_STBY, 1);

    ESP_LOGI(TAG, "TB6612 就绪 PWMA=%d AIN1=%d AIN2=%d STBY=%d, %dHz/10bit",
             PIN_PWMA, PIN_AIN1, PIN_AIN2, PIN_STBY, MOTOR_PWM_FREQ_HZ);
    return ESP_OK;
}

esp_err_t motor_set_dir(bool forward)
{
    s_forward = forward;
    apply_dir();
    ESP_LOGI(TAG, "方向: %s", forward ? "正转" : "反转");
    return ESP_OK;
}

esp_err_t motor_set_duty(float duty)
{
    if (duty < 0.0f || duty > 1.0f) {
        ESP_LOGW(TAG, "占空比 %.2f 越界，已钳到 0~1", duty);
        duty = duty < 0.0f ? 0.0f : 1.0f;
    }
    s_duty = duty;
    apply_duty();
    return ESP_OK;
}

esp_err_t motor_stop(void)
{
    s_duty = 0.0f;
    apply_duty();
    return ESP_OK;
}
