#include "sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "steps/steps.h"

static const char *TAG = "SLEEP";

static uint64_t build_wakeup_mask(const sleep_ctx_t *ctx) {
    uint64_t mask = 0;
    if (esp_sleep_is_valid_wakeup_gpio(ctx->button_gpio_1)) {
        rtc_gpio_init(ctx->button_gpio_1);
        rtc_gpio_set_direction(ctx->button_gpio_1, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(ctx->button_gpio_1);
        rtc_gpio_pulldown_dis(ctx->button_gpio_1);
        mask |= (1ULL << ctx->button_gpio_1);
    }
    if (esp_sleep_is_valid_wakeup_gpio(ctx->button_gpio_2)) {
        rtc_gpio_init(ctx->button_gpio_2);
        rtc_gpio_set_direction(ctx->button_gpio_2, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(ctx->button_gpio_2);
        rtc_gpio_pulldown_dis(ctx->button_gpio_2);
        mask |= (1ULL << ctx->button_gpio_2);
    }
    return mask;
}

void sleep_enter_deep(const sleep_ctx_t *ctx) {
    if (!ctx) return;
    uint64_t mask = build_wakeup_mask(ctx);
    if (mask == 0) {
        ESP_LOGE(TAG, "No valid wakeup GPIOs configured, aborting sleep");
        return;
    }
    while (gpio_get_level(ctx->button_gpio_1) == 0 || gpio_get_level(ctx->button_gpio_2) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW));
    if (ctx->pin_bl >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_bl, 0);
    }
    if (ctx->pin_pwr >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_pwr, 0);
    }
    esp_deep_sleep_start();
}

void sleep_enter_fast(const sleep_ctx_t *ctx) {
    if (!ctx) return;
    uint64_t mask = build_wakeup_mask(ctx);
    if (mask == 0) {
        ESP_LOGE(TAG, "No valid wakeup GPIOs configured, aborting fast sleep");
        return;
    }
    while (gpio_get_level(ctx->button_gpio_1) == 0 || gpio_get_level(ctx->button_gpio_2) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    step_wss_stop();
    if (ctx->wifi_connected) {
        *ctx->wifi_connected = false;
    }
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    esp_err_t wifi_err = esp_wifi_stop();
    if (wifi_err != ESP_OK && wifi_err != ESP_ERR_WIFI_NOT_INIT && wifi_err != ESP_ERR_WIFI_STOP_STATE) {
        ESP_LOGW(TAG, "Wi-Fi stop failed: %s", esp_err_to_name(wifi_err));
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW));
    if (ctx->pin_bl >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_bl, 0);
    }
    if (ctx->pin_pwr >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_pwr, 0);
    }
    esp_light_sleep_start();
    if (ctx->pin_pwr >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_pwr, 1);
    }
    if (ctx->panel) {
        ESP_ERROR_CHECK(esp_lcd_panel_reset(ctx->panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(ctx->panel));
        ESP_ERROR_CHECK(esp_lcd_panel_set_gap(ctx->panel, ctx->lcd_x_offset, ctx->lcd_y_offset));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(ctx->panel, true));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(ctx->panel, true));
    } else {
        ESP_LOGW(TAG, "No panel handle to re-init after fast sleep");
    }
    esp_err_t wifi_start_err = esp_wifi_start();
    if (wifi_start_err != ESP_OK) {
        ESP_LOGW(TAG, "Wi-Fi start after fast sleep failed: %s", esp_err_to_name(wifi_start_err));
    } else {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
    if (ctx->pin_bl >= 0) {
        gpio_set_level((gpio_num_t)ctx->pin_bl, 1);
    }
    if (ctx->last_ws_start_attempt) {
        *ctx->last_ws_start_attempt = 0;
    }
    if (ctx->button_queue) {
        xQueueReset(ctx->button_queue);
    }
}
