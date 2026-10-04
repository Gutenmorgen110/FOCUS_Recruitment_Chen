idf_component_register(SRCS "TASK3_LED.c"
                    REQUIRES esp_driver_uart esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS ".")
