idf_component_register(SRCS "led_ctrl.c"
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio
                    INCLUDE_DIRS "include")
                    
