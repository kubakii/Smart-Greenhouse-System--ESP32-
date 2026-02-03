idf_component_register(
            SRCS "mqtt_client.c"
            INCLUDE_DIRS "."
            REQUIRES    mqtt 
                        esp_netif 
                        json
                        uart_manager
                        wifi_manager
                    )
