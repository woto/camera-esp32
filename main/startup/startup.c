#include "startup.h"
#include "esp_log.h"
#include "net/wifi_ctrl.h"
#include "steps/steps.h"
#include "freertos/task.h"

static const char *TAG = "STARTUP";

static void show_digest_error(uint16_t *out_buf, esp_lcd_panel_handle_t panel_handle, bool wifi_connected, bool ws_connected, int error_pause_ms) {
    if (out_buf) {
        render_status_screen(panel_handle, out_buf, "Digest error", "No data", 0x0000, 0xF800, wifi_connected, ws_connected);
        vTaskDelay(pdMS_TO_TICKS(error_pause_ms));
    }
}

void startup_init(startup_ctx_t *ctx,
                  uint16_t *out_buf,
                  size_t out_buf_size,
                  esp_lcd_panel_handle_t panel_handle,
                  const char *wifi_ssid,
                  const char *wifi_pass,
                  int debug_delay_ms,
                  int error_pause_ms) {
    if (!ctx) {
        return;
    }
    *ctx = (startup_ctx_t){0};
    ctx->step = START_STEP_WIFI;
    step_wifi_begin(&ctx->steps_state, out_buf);

    // Wi-Fi
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    if (out_buf) {
        render_status_screen(panel_handle, out_buf, "Wi-Fi", NULL, 0x0000, 0xFFFF, wifi_ctrl_is_connected(), step_wss_is_connected());
    }
    vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
    wifi_ctrl_init(wifi_ssid, wifi_pass);
    EventBits_t wifi_bits = xEventGroupWaitBits(
        wifi_ctrl_event_group(),
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(10000)
    );
    bool wifi_connected = (wifi_bits & WIFI_CONNECTED_BIT) != 0;
    if (wifi_connected) {
        ESP_LOGI(TAG, "Wi-Fi Connected");
        ctx->step = START_STEP_SNTP;
        ctx->steps_state.wifi_connected = true;
        ctx->steps_state.step = START_STEP_SNTP;
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timeout; waiting for Wi-Fi");
        if (out_buf) {
            render_status_screen(panel_handle, out_buf, "Wi-Fi error", "No link", 0x0000, 0xF800, wifi_ctrl_is_connected(), step_wss_is_connected());
            vTaskDelay(pdMS_TO_TICKS(error_pause_ms));
        }
        ctx->step = START_STEP_WIFI;
        step_wifi_retry(&ctx->steps_state, out_buf);
    }

    // SNTP
    if (ctx->step == START_STEP_SNTP) {
        if (wifi_connected && step_sntp_run(out_buf, wifi_connected, debug_delay_ms, error_pause_ms)) {
            vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
            ctx->step = START_STEP_WSS;
            ctx->steps_state.time_synced = true;
            ctx->steps_state.step = START_STEP_WSS;
        } else {
            ctx->time_sync_pending = true;
        }
    }

    // Digest initial flags
    ctx->digest_pending = (ctx->step != START_STEP_READY);
    ctx->digest_wait_wifi = !wifi_connected;
    ctx->images_count = 0;
    ctx->current_image = 0;

    // Optional first digest attempt if we are already on Digest step (e.g., WS connected early)
    if (ctx->step == START_STEP_DIGEST) {
        step_digest_begin(&ctx->steps_state, out_buf);
        ctx->steps_state.wifi_connected = wifi_connected;
        if (wifi_connected) {
            int count = 0;
            if (step_digest_attempt(&ctx->steps_state, out_buf, &count)) {
                ctx->images_count = count;
                ctx->step = ctx->steps_state.step;
                ctx->digest_pending = ctx->steps_state.digest_pending;
            } else {
                ctx->digest_wait_wifi = true;
                ctx->digest_pending = true;
                show_digest_error(out_buf, panel_handle, wifi_connected, step_wss_is_connected(), error_pause_ms);
            }
        }
    } else if (ctx->step != START_STEP_READY) {
        step_digest_begin(&ctx->steps_state, out_buf);
    }
}

static void handle_digest_attempt(startup_ctx_t *ctx,
                                  uint16_t *out_buf,
                                  size_t out_buf_size,
                                  esp_lcd_panel_handle_t panel_handle,
                                  bool wifi_connected,
                                  bool menu_visible,
                                  image_slot_t *images,
                                  TickType_t *last_switch_tick,
                                  int *current_image,
                                  int debug_delay_ms,
                                  int error_pause_ms) {
    if (!ctx || !images || !current_image || !last_switch_tick) {
        return;
    }
    if (ctx->step != START_STEP_DIGEST || !ctx->digest_pending || !wifi_connected || ctx->digest_wait_wifi) {
        return;
    }

    if (out_buf) {
        render_status_screen(panel_handle, out_buf, "Digest", NULL, 0x0000, 0xFFFF, wifi_connected, step_wss_is_connected());
    }
    vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));

    ctx->steps_state.wifi_connected = wifi_connected;
    int new_count = ctx->images_count;
    bool ok = step_digest_attempt(&ctx->steps_state, out_buf, &new_count);
    ctx->digest_pending = ctx->steps_state.digest_pending;
    if (ok && new_count > 0) {
        ctx->images_count = new_count;
        ctx->current_image = 0;
        ctx->step = ctx->steps_state.step;
        if (!menu_visible && out_buf && images[0].valid) {
            if (images_fetch_and_show(&images[0], out_buf, out_buf_size, panel_handle, wifi_connected, step_wss_is_connected())) {
                *last_switch_tick = xTaskGetTickCount();
                *current_image = (new_count > 1) ? 1 : 0;
            } else {
                ESP_LOGE(TAG, "Failed to display refreshed image 0");
                ctx->step = START_STEP_DIGEST;
            }
        }
    } else {
        show_digest_error(out_buf, panel_handle, wifi_connected, step_wss_is_connected(), error_pause_ms);
        ctx->digest_pending = true;
        ctx->digest_wait_wifi = true;
    }
}

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
                         int error_pause_ms) {
    if (!ctx) return;

    ctx->steps_state.wifi_connected = wifi_connected;

    if (ctx->step == START_STEP_WSS && step_wss_is_connected()) {
        ctx->step = START_STEP_DIGEST;
        ctx->digest_pending = true;
        ctx->digest_wait_wifi = false;
    }

    if (wifi_state_changed && !wifi_connected) {
        ctx->digest_wait_wifi = true;
    }

    handle_digest_attempt(ctx, out_buf, out_buf_size, panel_handle, wifi_connected, menu_visible, images, last_switch_tick, current_image, debug_delay_ms, error_pause_ms);

    if (wifi_just_connected) {
        if (ctx->step == START_STEP_WIFI) {
            ctx->step = START_STEP_SNTP;
        }
        if (ctx->time_sync_pending && !step_sntp_is_synced() && wifi_connected) {
            if (out_buf) {
                render_status_screen(panel_handle, out_buf, "Time", NULL, 0x0000, 0xFFFF, wifi_connected, step_wss_is_connected());
            }
            vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
            if (step_sntp_run(out_buf, wifi_connected, debug_delay_ms, error_pause_ms)) {
                vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
                ctx->time_sync_pending = false;
                ctx->step = START_STEP_WSS;
            } else {
                ctx->time_sync_pending = true;
                ctx->step = START_STEP_SNTP;
            }
        }

        if (ctx->step == START_STEP_DIGEST || ctx->digest_pending) {
            ctx->digest_wait_wifi = false;
            handle_digest_attempt(ctx, out_buf, out_buf_size, panel_handle, wifi_connected, menu_visible, images, last_switch_tick, current_image, debug_delay_ms, error_pause_ms);
        }
    }

    if (step_wss_take_feed_reload()) {
        ESP_LOGI(TAG, "Reloading feed after upload_success notification");
        if (out_buf) {
            render_status_screen(panel_handle, out_buf, "Digest", NULL, 0x0000, 0xFFFF, wifi_connected, step_wss_is_connected());
        }
        if (!wifi_connected) {
            ctx->digest_pending = true;
            ctx->step = START_STEP_DIGEST;
            step_digest_begin(&ctx->steps_state, out_buf);
        } else {
            vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
            ctx->steps_state.wifi_connected = wifi_connected;
            int new_count = ctx->images_count;
            bool ok = step_digest_attempt(&ctx->steps_state, out_buf, &new_count);
            ctx->digest_pending = ctx->steps_state.digest_pending;
            if (ok && new_count > 0) {
                ctx->images_count = new_count;
                ctx->current_image = 0;
                ctx->step = ctx->steps_state.step;
                if (!menu_visible && out_buf && images[0].valid) {
                    if (images_fetch_and_show(&images[0], out_buf, out_buf_size, panel_handle, wifi_connected, step_wss_is_connected())) {
                        *last_switch_tick = xTaskGetTickCount();
                        *current_image = (new_count > 1) ? 1 : 0;
                    } else {
                        ESP_LOGE(TAG, "Failed to display refreshed image 0");
                        ctx->step = START_STEP_DIGEST;
                    }
                }
            } else {
                ESP_LOGW(TAG, "Feed reload returned no images; keeping existing list");
                show_digest_error(out_buf, panel_handle, wifi_connected, step_wss_is_connected(), error_pause_ms);
            }
        }
    }
}
