set(src_dirs LED MOTOR MOTOR_CTRL ENCODER PID UART WIFI HTTP BLE CTRL)

idf_component_register(SRC_DIRS ${src_dirs}
                       INCLUDE_DIRS ${src_dirs}
                       PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart esp_driver_pcnt esp_wifi esp_event esp_http_server nvs_flash
                       EMBED_FILES "HTTP/index.html")
