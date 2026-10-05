/* HC-04D 走 UART 透传，不启用 ESP32 蓝牙协议栈 */
#include "esp_err.h"
#include "esp_log.h"
#include "uart_cmd.h"
#include "app_ctrl.h"
#include "bt_hc04.h"

static const char *TAG = "bt";

/* 用 UART1 的 17/18；43/44 上焊着板载 USB 桥接芯片 */
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

    uart_cmd_reply(BT_UART, "READY  (HELP 看指令)");
    ESP_LOGI(TAG, "HC-04D 就绪 TX=%d RX=%d %d 8N1（接线要交叉，且必须共地）",
             BT_TX_PIN, BT_RX_PIN, CONFIG_BT_UART_BAUD);
    return ESP_OK;
}
