#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"

typedef struct {
    int button_gpio_1;
    int button_gpio_2;
    int pin_bl;
    int pin_pwr;
    int lcd_x_offset;
    int lcd_y_offset;
    esp_lcd_panel_handle_t panel;
    QueueHandle_t button_queue;
    volatile bool *wifi_connected;
} sleep_ctx_t;

void sleep_enter_deep(const sleep_ctx_t *ctx);
void sleep_enter_fast(const sleep_ctx_t *ctx);
