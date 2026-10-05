#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* 连上路由器后自动拉起 HTTP 服务 */
esp_err_t wifi_init_sta(void);

#ifdef __cplusplus
}
#endif
