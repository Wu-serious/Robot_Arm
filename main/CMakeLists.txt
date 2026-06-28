idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES Middleware Communication nvs_flash esp_wifi esp_wifi_remote)
