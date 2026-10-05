/* 一个 LED 引脚三种驱法，切换时要把引脚从 LEDC 的矩阵路由上摘下来再重绑 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led.h"

static const char *TAG = "led";

#define LED_GPIO        GPIO_NUM_8
#define LED_LS_MODE     LEDC_LOW_SPEED_MODE
#define LED_TIMER       LEDC_TIMER_0
#define LED_CH          LEDC_CHANNEL_0
#define LED_DUTY_RES    LEDC_TIMER_13_BIT
#define LED_FREQ_HZ     5000        /* 人眼无频闪 */
#define LED_DUTY_MAX    ((1 << 13) - 1)

#define LED_BLINK_MS    500
#define LED_BREATH_MS   2000

static const ledc_channel_config_t LED_CH_CFG = {
    .channel    = LED_CH,
    .duty       = 0,
    .gpio_num   = LED_GPIO,
    .speed_mode = LED_LS_MODE,
    .hpoint     = 0,
    .timer_sel  = LED_TIMER,
    .flags.output_invert = 0,
};

static led_mode_t s_mode = LED_MODE_BLINK;
static bool s_on;
static int  s_pct = 100;
static bool s_blink_level;
static SemaphoreHandle_t s_fade_done;
static SemaphoreHandle_t s_breath_go;

static IRAM_ATTR bool fade_end_cb(const ledc_cb_param_t *param, void *arg)
{
    BaseType_t woken = pdFALSE;
    if (param->event == LEDC_FADE_END_EVT) {
        xSemaphoreGiveFromISR(s_fade_done, &woken);
    }
    return woken == pdTRUE;
}

static void bind_gpio(void)
{
    ledc_stop(LED_LS_MODE, LED_CH, 0);
    xSemaphoreGive(s_fade_done);        /* 呼吸任务可能正卡在等 fade 结束 */
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

static void bind_ledc(void)
{
    gpio_reset_pin(LED_GPIO);
    ledc_channel_config(&LED_CH_CFG);
}

static void apply_manual(void)
{
    uint32_t duty = s_on ? (uint32_t)s_pct * LED_DUTY_MAX / 100 : 0;
    ledc_set_duty_and_update(LED_LS_MODE, LED_CH, duty, 0);
}

static void ensure_manual(void)
{
    if (s_mode == LED_MODE_MANUAL) {
        apply_manual();
    } else {
        led_set_mode(LED_MODE_MANUAL);      /* 里面会 apply */
    }
}

static void blink_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(LED_BLINK_MS));
        if (s_mode != LED_MODE_BLINK) {
            continue;
        }
        s_blink_level = !s_blink_level;
        gpio_set_level(LED_GPIO, s_blink_level);
    }
}

static void breath_task(void *arg)
{
    for (;;) {
        xSemaphoreTake(s_breath_go, portMAX_DELAY);
        while (s_mode == LED_MODE_BREATH) {
            ledc_set_fade_with_time(LED_LS_MODE, LED_CH, LED_DUTY_MAX, LED_BREATH_MS);
            ledc_fade_start(LED_LS_MODE, LED_CH, LEDC_FADE_NO_WAIT);
            xSemaphoreTake(s_fade_done, portMAX_DELAY);
            if (s_mode != LED_MODE_BREATH) {
                break;
            }

            ledc_set_fade_with_time(LED_LS_MODE, LED_CH, 0, LED_BREATH_MS);
            ledc_fade_start(LED_LS_MODE, LED_CH, LEDC_FADE_NO_WAIT);
            xSemaphoreTake(s_fade_done, portMAX_DELAY);
        }
    }
}

esp_err_t led_init(void)
{
    s_fade_done = xSemaphoreCreateBinary();
    s_breath_go = xSemaphoreCreateBinary();

    ledc_timer_config_t t = {
        .duty_resolution = LED_DUTY_RES,
        .freq_hz         = LED_FREQ_HZ,
        .speed_mode      = LED_LS_MODE,
        .timer_num       = LED_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ESP_ERROR_CHECK(ledc_channel_config(&LED_CH_CFG));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    ledc_cbs_t cbs = { .fade_cb = fade_end_cb };
    ESP_ERROR_CHECK(ledc_cb_register(LED_LS_MODE, LED_CH, &cbs, NULL));

    xTaskCreate(blink_task, "led_blink", 2560, NULL, 5, NULL);
    xTaskCreate(breath_task, "led_breath", 3072, NULL, 5, NULL);

    /* 初始是 GPIO 闪烁，得把引脚从刚配好的 LEDC 上摘下来，否则 gpio_set_level 不生效 */
    s_mode = LED_MODE_BLINK;
    bind_gpio();

    ESP_LOGI(TAG, "LED 就绪 GPIO%d %dHz/13bit", LED_GPIO, LED_FREQ_HZ);
    return ESP_OK;
}

esp_err_t led_set_mode(led_mode_t mode)
{
    if (mode == s_mode) {
        return ESP_OK;
    }
    s_mode = mode;

    switch (mode) {
    case LED_MODE_BLINK:
        bind_gpio();
        s_blink_level = false;
        gpio_set_level(LED_GPIO, 0);
        break;
    case LED_MODE_BREATH:
        bind_ledc();
        xSemaphoreGive(s_breath_go);
        break;
    case LED_MODE_MANUAL:
        bind_ledc();
        apply_manual();
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "模式 → %s",
             mode == LED_MODE_BLINK ? "BLINK" : mode == LED_MODE_BREATH ? "BREATH" : "MANUAL");
    return ESP_OK;
}

led_mode_t led_get_mode(void)
{
    return s_mode;
}

void led_on(void)
{
    if (s_pct == 0) {
        s_pct = 100;        /* 之前在 0%，ON 就按全亮 */
    }
    s_on = true;
    ensure_manual();
}

void led_off(void)
{
    s_on = false;
    ensure_manual();
}

void led_set_brightness(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    s_pct = percent;
    s_on = percent > 0;
    ensure_manual();
}

bool led_is_on(void)
{
    return s_on;
}

int led_get_brightness(void)
{
    return s_pct;
}
