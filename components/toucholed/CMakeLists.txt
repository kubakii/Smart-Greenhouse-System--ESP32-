idf_component_register(
            SRCS "ft6336u.c" "st7789.c"
            INCLUDE_DIRS "."
            REQUIRES    driver
                        freertos
                        esp_timer
                        esp_common
                        heap
                    )
