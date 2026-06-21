idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES Middleware Communication bt nvs_flash esp_wifi_remote)
