#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "esp_err.h"

esp_err_t encoder_init(void);

float encoder_get_rpm(void);
bool encoder_is_ready(void);

#ifdef __cplusplus
}
#endif
