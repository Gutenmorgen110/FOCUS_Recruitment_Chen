/* 网页控制：api/cmd 是通用口，把 URL 里的指令原样丢给 app_ctrl；
 * led/on、led/off、led/speed 是原来点灯那版留下来的，保持能用。
 */
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "led.h"
#include "app_ctrl.h"
#include "http.h"

static const char *TAG = "http";
static httpd_handle_t s_server;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

/* GET /api/cmd?c=MOTOR%20RPM%20250 */
static esp_err_t cmd_handler(httpd_req_t *req)
{
    char query[256];
    char cmd[128];
    char resp[192];

    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1 || qlen > sizeof(query)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
    }
    if (httpd_req_get_url_query_str(req, query, qlen) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
    }
    if (httpd_query_key_value(query, "c", cmd, sizeof(cmd)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing c");
    }

    app_ctrl_exec(cmd, resp, sizeof(resp));
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, resp[0] ? resp : "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t led_on_handler(httpd_req_t *req)
{
    led_on();
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t led_off_handler(httpd_req_t *req)
{
    led_off();
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t led_speed_handler(httpd_req_t *req)
{
    char query[32];
    char param[8];

    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1 || qlen > sizeof(query)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
    }
    if (httpd_req_get_url_query_str(req, query, qlen) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
    }
    if (httpd_query_key_value(query, "value", param, sizeof(param)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing value");
    }

    int val = atoi(param);
    if (val < 0 || val > 100) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Value out of range");
    }
    led_set_brightness(val);
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t URIS[] = {
    { .uri = "/",          .method = HTTP_GET, .handler = index_handler },
    { .uri = "/api/cmd",   .method = HTTP_GET, .handler = cmd_handler },
    { .uri = "/led/on",    .method = HTTP_GET, .handler = led_on_handler },
    { .uri = "/led/off",   .method = HTTP_GET, .handler = led_off_handler },
    { .uri = "/led/speed", .method = HTTP_GET, .handler = led_speed_handler },
};

esp_err_t http_server_start(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 启动失败");
        s_server = NULL;
        return ESP_FAIL;
    }

    for (size_t i = 0; i < sizeof(URIS) / sizeof(URIS[0]); i++) {
        httpd_register_uri_handler(s_server, &URIS[i]);
    }

    ESP_LOGI(TAG, "HTTP 已启动，共 %u 个接口",
             (unsigned)(sizeof(URIS) / sizeof(URIS[0])));
    return ESP_OK;
}
