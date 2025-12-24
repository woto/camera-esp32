#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_ctrl_init(const char *ssid, const char *pass);
EventGroupHandle_t wifi_ctrl_event_group(void);
bool wifi_ctrl_is_connected(void);
bool wifi_ctrl_refresh_link(void);
esp_err_t wifi_ctrl_connect(void);
bool *wifi_ctrl_connected_ptr(void);
