#include "esp_err.h"
#include "esp_log.h"
#include "motor.h"
#include "motor_ctrl.h"
#include "cli.h"

static const char *TAG = "TASK5";

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(motor_ctrl_start());
    ESP_ERROR_CHECK(cli_start());

    ESP_LOGI(TAG, "上电即进入自动渐变：慢 → 中 → 快 → 中 → 慢");
    ESP_LOGI(TAG, "指令：ON / OFF / SPEED <0-100> / AUTO / DIR F|R / STATUS");
}
