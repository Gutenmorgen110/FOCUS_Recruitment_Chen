#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    LED_MODE_BLINK,     /* GPIO 方波翻转 */
    LED_MODE_BREATH,    /* 硬件淡入淡出 */
    LED_MODE_MANUAL,    /* 手动亮度 */
} led_mode_t;

esp_err_t led_init(void);

esp_err_t led_set_mode(led_mode_t mode);
led_mode_t led_get_mode(void);

void led_on(void);
void led_off(void);
void led_set_brightness(int percent);

bool led_is_on(void);
int led_get_brightness(void);

#ifdef __cplusplus
}
#endif
