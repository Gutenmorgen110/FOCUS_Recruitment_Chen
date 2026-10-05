#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "app_ctrl.h"
#include "uart_cmd.h"
#include "wifi.h"
#include "bt_hc04.h"

static const char *TAG = "main";

/* UART0 走板载桥接芯片，就是串口助手连的那个口 */
#define CONSOLE_TX  GPIO_NUM_43
#define CONSOLE_RX  GPIO_NUM_44
#define CONSOLE_BAUD 115200

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(app_ctrl_init());

    ESP_ERROR_CHECK(uart_cmd_init(UART_NUM_0, CONSOLE_TX, CONSOLE_RX,
                                  CONSOLE_BAUD, app_ctrl_on_line));
    uart_cmd_reply(UART_NUM_0, "READY  (HELP 看指令)");

    bt_hc04_init();
    wifi_init_sta();

    ESP_LOGI(TAG, "启动完成");
}
