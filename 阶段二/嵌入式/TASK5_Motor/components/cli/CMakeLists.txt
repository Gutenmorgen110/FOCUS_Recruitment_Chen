idf_component_register(SRCS "cli.c"
                    PRIV_REQUIRES esp_driver_uart esp_driver_gpio motor_ctrl
                    INCLUDE_DIRS "include")
