idf_component_register(
            SRCS "lvgl_driver.c"
            INCLUDE_DIRS "."
            REQUIRES    driver
                        esp_lcd 
                        lvgl
                    )