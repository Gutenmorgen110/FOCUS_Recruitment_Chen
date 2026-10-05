#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "app_ctrl.h"
#include "wifi.h"
#include "bt_hc04.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 灯/电机/编码器/PID 都在这里面初始化 */
    ESP_ERROR_CHECK(app_ctrl_init());

    if (bt_hc04_init() != ESP_OK) {
        ESP_LOGE(TAG, "蓝牙初始化失败");
    }

    /* 阻塞到连上或重试用尽 */
    wifi_init_sta();

    ESP_LOGI(TAG, "启动完成");
}
