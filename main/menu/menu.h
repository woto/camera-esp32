#pragma once

#include <stdint.h>
#include "esp_lcd_panel_ops.h"
#include "display_config.h"

#define MENU_ITEMS 4
#define MENU_ITEM_BATTERY 0
#define MENU_ITEM_DEVICE_ID 1
#define MENU_ITEM_FAST_SLEEP 2
#define MENU_ITEM_DEEP_SLEEP 3

void menu_render(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, int selected, int bat_mv, bool wifi_connected, bool ws_connected);
