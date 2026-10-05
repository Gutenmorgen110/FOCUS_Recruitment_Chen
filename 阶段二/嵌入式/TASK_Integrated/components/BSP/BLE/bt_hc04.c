/* HC-04D 就是一路 UART 透传，不用启用 ESP32 的蓝牙协议栈，
 * 省掉整个 Bluedroid/NimBLE 的 Flash 和 RAM，也避开跟 Wi-Fi 抢射频。
 */
#include "esp_err.h"
#include "esp_log.h"
#include "uart_cmd.h"
#include "app_ctrl.h"
#include "bt_hc04.h"

static const char *TAG = "bt";

#define BT_UART     UART_NUM_1
#define BT_TX_PIN   17      /* ESP32 TX → 模块 RXD */
#define BT_RX_PIN   18      /* ESP32 RX ← 模块 TXD */

esp_err_t bt_hc04_init(void)
{
    esp_err_t ret = uart_cmd_init(BT_UART, BT_TX_PIN, BT_RX_PIN,
                                  CONFIG_BT_UART_BAUD, app_ctrl_on_line);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "HC-04D 就绪 TX=%d RX=%d %d 8N1（接线要交叉，且必须共地）",
             BT_TX_PIN, BT_RX_PIN, CONFIG_BT_UART_BAUD);
    return ESP_OK;
}
