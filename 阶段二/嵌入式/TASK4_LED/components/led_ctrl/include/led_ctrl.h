#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LEDC 硬件
 *
 * 配置 LEDC 定时器和通道，初始状态为熄灭。
 * 必须在调用其他 led_ctrl_xxx 函数之前调用一次。
 */
void led_ctrl_init(void);

/**
 * @brief 点亮 LED
 *
 * 使用当前亮度值点亮，默认亮度 100%。
 */
void led_ctrl_on(void);

/**
 * @brief 熄灭 LED
 *
 * 占空比置 0，但不改变记录的亮度值。
 */
void led_ctrl_off(void);

/**
 * @brief 设置亮度
 *
 * @param percent 亮度百分比，0~100
 *                0 表示熄灭，100 表示最亮
 *                越界值会被忽略并打警告日志
 *
 * 设置后 LED 自动点亮。
 */
void led_ctrl_set_brightness(int percent);

#ifdef __cplusplus
}
#endif