#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
esp_err_t cli_start(void);
void cli_process_cmd(char *cmd);

#ifdef __cplusplus
}
#endif
