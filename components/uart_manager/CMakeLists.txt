idf_component_register(
            SRCS "uart.c"
            INCLUDE_DIRS "."
            REQUIRES    driver
                        freertos
                    
                    )
