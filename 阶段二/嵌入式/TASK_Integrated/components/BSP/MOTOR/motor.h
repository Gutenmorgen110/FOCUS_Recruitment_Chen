#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

esp_err_t motor_init(void);

/* forward=true 正转，false 反转 */
esp_err_t motor_set_dir(bool forward);

/* duty 取 0.0~1.0，越界钳回；只改速度不改方向 */
esp_err_t motor_set_duty(float duty);

/* 占空比清零，方向状态保留 */
esp_err_t motor_stop(void);

#ifdef __cplusplus
}
#endif
