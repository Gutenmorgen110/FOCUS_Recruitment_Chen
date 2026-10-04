/* HTTP Server for ESP32-S3 LED Control
 *
 * 提供 4 个接口：
 *   GET /                    返回控制页面
 *   GET /led/on              点亮 LED
 *   GET /led/off             熄灭 LED
 *   GET /led/speed?value=N   设置亮度 (0~100)
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"

#include "http_server.h"
#include "led_ctrl.h"

static const char *TAG = "http_server";
static httpd_handle_t s_server = NULL;

/* ==================== 嵌入的 HTML ==================== */
/* 符号名由 CMakeLists.txt 里的 EMBED_FILES "index.html" 自动生成：
 *   _binary_index_html_start
 *   _binary_index_html_end
 */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* ==================== URI 处理函数 ==================== */

/* GET / : 返回控制页面 */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

/* GET /led/on : 点亮 */
static esp_err_t led_on_handler(httpd_req_t *req)
{
    led_ctrl_on();
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

/* GET /led/off : 熄灭 */
static esp_err_t led_off_handler(httpd_req_t *req)
{
    led_ctrl_off();
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

/* GET /led/speed?value=50 : 设置亮度 */
static esp_err_t led_speed_handler(httpd_req_t *req)
{
    char query[32];
    size_t query_len = httpd_req_get_url_query_len(req) + 1;

    if (query_len <= 1 || query_len > sizeof(query)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
    }

    if (httpd_req_get_url_query_str(req, query, query_len) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
    }

    char param[8];
    if (httpd_query_key_value(query, "value", param, sizeof(param)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing value");
    }

    int val = atoi(param);
    if (val < 0 || val > 100) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Value out of range");
    }

    led_ctrl_set_brightness(val);
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

/* ==================== URI 定义 ==================== */

static const httpd_uri_t uri_index = {
    .uri     = "/",
    .method  = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t uri_led_on = {
    .uri     = "/led/on",
    .method  = HTTP_GET,
    .handler = led_on_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t uri_led_off = {
    .uri     = "/led/off",
    .method  = HTTP_GET,
    .handler = led_off_handler,
    .user_ctx = NULL,
};

static const httpd_uri_t uri_led_speed = {
    .uri     = "/led/speed",
    .method  = HTTP_GET,
    .handler = led_speed_handler,
    .user_ctx = NULL,
};

/* ==================== 启动函数 ==================== */

void http_server_start(void)
{
    if (s_server != NULL) {
        ESP_LOGW(TAG, "HTTP server already running");
        return;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        s_server = NULL;
        return;
    }

    httpd_register_uri_handler(s_server, &uri_index);
    httpd_register_uri_handler(s_server, &uri_led_on);
    httpd_register_uri_handler(s_server, &uri_led_off);
    httpd_register_uri_handler(s_server, &uri_led_speed);

    ESP_LOGI(TAG, "HTTP server started, 4 URI handlers registered");
}