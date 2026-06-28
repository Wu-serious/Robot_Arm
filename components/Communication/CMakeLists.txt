set(srcs
        WiFi/WiFi.c
        WebServer/WebServer.c)

set(include_dirs
        WiFi
        WebServer)

set(requires
        driver
        esp_wifi
        esp_wifi_remote
        esp_event
        nvs_flash
        esp_netif
        esp_http_server
        json
        lwip)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    REQUIRES "${requires}")
component_compile_options(-O2 -Wno-error=format -Wno-format)
