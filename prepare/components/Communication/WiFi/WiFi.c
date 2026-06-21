#include "WiFi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi_remote.h"

static const char *TAG = "WiFi_AP";

#define WIFI_AP_STARTED_BIT BIT0
#define WIFI_STA_CONNECTED_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_ap_netif = NULL;
static wifi_status_t s_wifi_status = {0};
static char s_ssid[32] = {0};
static char s_password[64] = {0};
static int s_num_stations = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START)
    {
        s_wifi_status.state = WIFI_STATE_STARTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
        ESP_LOGI(TAG, "WiFi AP started");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP)
    {
        s_wifi_status.state = WIFI_STATE_IDLE;
        xEventGroupClearBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
        ESP_LOGI(TAG, "WiFi AP stopped");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        s_num_stations++;
        s_wifi_status.state = WIFI_STATE_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
        ESP_LOGI(TAG, "Station connected: MAC="MACSTR", AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        s_num_stations--;
        if (s_num_stations == 0)
        {
            s_wifi_status.state = WIFI_STATE_STARTED;
            xEventGroupClearBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Station disconnected: MAC="MACSTR", AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

esp_err_t WiFi_AP_Init(const char *ssid, const char *password)
{
    esp_err_t ret = ESP_OK;

    if (ssid == NULL)
    {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    strncpy(s_wifi_status.ssid, ssid, sizeof(s_wifi_status.ssid) - 1);
    
    if (password != NULL)
    {
        strncpy(s_password, password, sizeof(s_password) - 1);
    }
    else
    {
        s_password[0] = '\0';
    }

    ESP_LOGI(TAG, "Initializing WiFi AP via ESP32-C5 co-processor...");

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_netif_init();

    s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_remote_init(&cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init remote WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_remote_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_event_handler_instance_t instance_any_id;
    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                                ESP_EVENT_ANY_ID,
                                                &wifi_event_handler,
                                                NULL,
                                                &instance_any_id);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = 1,
            .max_connection = 4,
            .authmode = (password && strlen(password) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
        }
    };
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    if (password && strlen(password) > 0)
    {
        strncpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password) - 1);
    }

    ret = esp_wifi_remote_set_config(ESP_IF_WIFI_AP, &wifi_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_ip_info_t ip_info = {
        .ip = { .addr = ESP_IP4TOADDR(192, 168, 4, 1) },
        .netmask = { .addr = ESP_IP4TOADDR(255, 255, 255, 0) },
        .gw = { .addr = ESP_IP4TOADDR(192, 168, 4, 1) },
    };
    ret = esp_netif_set_ip_info(s_ap_netif, &ip_info);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set IP info: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set DNS server to 192.168.4.1 for captive portal */
    esp_netif_dns_info_t dns_info;
    memset(&dns_info, 0, sizeof(dns_info));
    dns_info.ip.type = 0; /* IPADDR_TYPE_V4 */
    dns_info.ip.u_addr.ip4.addr = ESP_IP4TOADDR(192, 168, 4, 1);
    esp_netif_set_dns_info(s_ap_netif, ESP_NETIF_DNS_MAIN, &dns_info);

    esp_netif_dhcps_start(s_ap_netif);

    ret = esp_wifi_remote_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    strcpy(s_wifi_status.ip_address, "192.168.4.1");
    s_wifi_status.state = WIFI_STATE_STARTED;

    ESP_LOGI(TAG, "WiFi AP started with SSID: %s, IP: 192.168.4.1", ssid);
    return ESP_OK;
}

esp_err_t WiFi_AP_Deinit(void)
{
    esp_err_t ret = ESP_OK;

    ret = esp_wifi_remote_stop();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_remote_deinit();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deinit WiFi: %s", esp_err_to_name(ret));
    }

    if (s_wifi_event_group != NULL)
    {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    if (s_ap_netif != NULL)
    {
        esp_netif_destroy(s_ap_netif);
        s_ap_netif = NULL;
    }

    s_wifi_status.state = WIFI_STATE_IDLE;
    s_num_stations = 0;
    memset(s_wifi_status.ip_address, 0, sizeof(s_wifi_status.ip_address));

    ESP_LOGI(TAG, "WiFi AP deinitialized");
    return ret;
}

esp_err_t WiFi_AP_GetStatus(wifi_status_t *status)
{
    if (status == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(status, &s_wifi_status, sizeof(wifi_status_t));
    status->num_stations = s_num_stations;

    return ESP_OK;
}

bool WiFi_AP_IsStarted(void)
{
    return s_wifi_status.state == WIFI_STATE_STARTED || 
           s_wifi_status.state == WIFI_STATE_CONNECTED;
}

bool WiFi_AP_HasStations(void)
{
    return s_num_stations > 0;
}