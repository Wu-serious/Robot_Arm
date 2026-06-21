#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_STARTED,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR
} wifi_state_t;

typedef struct {
    wifi_state_t state;
    char ssid[32];
    char ip_address[16];
    int num_stations;
} wifi_status_t;

esp_err_t WiFi_AP_Init(const char *ssid, const char *password);
esp_err_t WiFi_AP_Deinit(void);
esp_err_t WiFi_AP_GetStatus(wifi_status_t *status);
bool WiFi_AP_IsStarted(void);
bool WiFi_AP_HasStations(void);

#ifdef __cplusplus
}
#endif

#endif