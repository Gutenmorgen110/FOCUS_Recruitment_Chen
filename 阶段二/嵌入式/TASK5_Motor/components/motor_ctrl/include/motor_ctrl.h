#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

/** 电机运行模式 */
typedef enum {
    MOTOR_MODE_STOP = 0,   /**< 停止，占空比 0 */
    MOTOR_MODE_SWEEP,      /**< 自动渐变：慢 → 中 → 快 → 中 循环 */
    MOTOR_MODE_MANUAL,    
} motor_mode_t;

esp_err_t motor_ctrl_start(void);

void motor_ctrl_set_manual(int percent);

void motor_ctrl_set_sweep(void);

void motor_ctrl_stop(void);

void motor_ctrl_resume(void);

void motor_ctrl_set_dir(bool forward);

motor_mode_t motor_ctrl_get_mode(void);

int motor_ctrl_get_target(void);

#ifdef __cplusplus
}
#endif
