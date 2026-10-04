idf_component_register(SRCS "TASK4_LED.c"
                    PRIV_REQUIRES esp_wifi nvs_flash http_server led_ctrl
                    INCLUDE_DIRS ".")
