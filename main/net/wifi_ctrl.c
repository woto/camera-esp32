#include "wifi_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <string.h>

static const char *TAG = "WIFI-CTRL";

static EventGroupHandle_t s_wifi_event_group = NULL;
static volatile bool s_wifi_connected = false;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_driver_connected(void) {
    wifi_ap_record_t ap = {0};
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

void wifi_ctrl_init(const char *ssid, const char *pass) {
    if (s_wifi_event_group) {
        return;
    }
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    if (ssid) {
        strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    }
    if (pass) {
        strlcpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Keep Wi-Fi awake during transfers to reduce TLS timeouts
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "wifi_ctrl_init finished.");
}

EventGroupHandle_t wifi_ctrl_event_group(void) {
    return s_wifi_event_group;
}

bool wifi_ctrl_is_connected(void) {
    return s_wifi_connected;
}

bool wifi_ctrl_refresh_link(void) {
    bool driver_connected = wifi_driver_connected();
    if (driver_connected && !s_wifi_connected) {
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (!driver_connected && s_wifi_connected) {
        s_wifi_connected = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    return s_wifi_connected;
}

esp_err_t wifi_ctrl_connect(void) {
    return esp_wifi_connect();
}

bool *wifi_ctrl_connected_ptr(void) {
    return (bool *)&s_wifi_connected;
}
