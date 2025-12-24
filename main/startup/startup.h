#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "steps/steps.h"
#include "images/images.h"
#include "render/render.h"
#include "esp_lcd_panel_interface.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    startup_step_t step;
    steps_state_t steps_state;
    bool time_sync_pending;
    bool digest_pending;
    bool digest_wait_wifi;
    int images_count;
    int current_image;
} startup_ctx_t;

void startup_init(startup_ctx_t *ctx,
                  uint16_t *out_buf,
                  size_t out_buf_size,
                  esp_lcd_panel_handle_t panel_handle,
                  const char *wifi_ssid,
                  const char *wifi_pass,
                  int debug_delay_ms,
                  int error_pause_ms);

void startup_handle_loop(startup_ctx_t *ctx,
                         bool wifi_connected,
                         bool wifi_state_changed,
                         bool wifi_just_connected,
                         bool ws_state_changed,
                         bool menu_visible,
                         image_slot_t *images,
                         uint16_t *out_buf,
                         size_t out_buf_size,
                         esp_lcd_panel_handle_t panel_handle,
                         TickType_t *last_switch_tick,
                         int *current_image,
                         int debug_delay_ms,
                         int error_pause_ms);
