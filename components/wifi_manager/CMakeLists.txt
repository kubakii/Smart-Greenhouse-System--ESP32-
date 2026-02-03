idf_component_register(
            SRCS "wifi_manager.c"
            INCLUDE_DIRS "."
            REQUIRES    driver
                        esp_wifi
                        esp_event
                        esp_netif
                        nvs_flash
                        lwip
                        freertos
                    
                    )
                    
