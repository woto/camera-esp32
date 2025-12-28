#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_sntp.h"
#include "steps/steps.h"
#include "net/wifi_ctrl.h"
#include "driver/gpio.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_attr.h"
#include "tls/certs.h"
#include "sleep/sleep.h"
#include "display_config.h"
#include "menu/menu.h"
#include "icons/icons.h"
#include "render/render.h"
#include "startup/startup.h"
#include "net/wifi_ctrl.h"
#include "net/trigger.h"
#include "net/ws_handler.h"
#include "images/images.h"
#include "power/battery.h"

// JPEG Decoder
#include "jpeg_decoder.h"

// --- CONFIG ---
#define WIFI_SSID      "AndroidAP4484"
#define WIFI_PASS      "dflg7101"
#define API_URL        "https://volleycam.com/"

// --- Lilygo T-Display S3 (1.9" ST7789, 8-bit I80) Pinout ---
#define PIN_NUM_D0   39
#define PIN_NUM_D1   40
#define PIN_NUM_D2   41
#define PIN_NUM_D3   42
#define PIN_NUM_D4   45
#define PIN_NUM_D5   46
#define PIN_NUM_D6   47
#define PIN_NUM_D7   48
#define PIN_NUM_WR   8
#define PIN_NUM_RD   9
#define PIN_NUM_DC   7
#define PIN_NUM_CS   6
#define PIN_NUM_RST  5
#define PIN_NUM_BL   38
#define PIN_NUM_PWR  15
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define LCD_DMA_ALIGN 16
#define LCD_DMA_CHUNK_BYTES 4080

#define BUTTON_GPIO 0
#define BUTTON_GPIO_2 14
#define BUTTON_DEBOUNCE_MS 250
#define IMAGE_ROTATION_INTERVAL_MS 1000
#define STATUS_ERROR_PAUSE_MS 1200
#define WIFI_RECONNECT_INTERVAL_MS 1000
#define STEP_DEBUG_PAUSE_MS 1000
#define WS_RETRY_INTERVAL_MS 5000

#define BAT_ADC_UNIT ADC_UNIT_1
#define BAT_ADC_CHANNEL ADC_CHANNEL_3
#define BAT_ADC_ATTEN ADC_ATTEN_DB_12
#define BAT_VOLT_DIVIDER_NUM 2
#define BAT_VOLT_DIVIDER_DEN 1

static const char *TAG = "NET-IMG";

// Wi-Fi control aliases
#define s_wifi_connected (*wifi_ctrl_connected_ptr())
#define s_wifi_event_group (wifi_ctrl_event_group())
static inline void wifi_init_sta(void) { wifi_ctrl_init(WIFI_SSID, WIFI_PASS); }

// --- Wi-Fi ---
static QueueHandle_t s_button_queue = NULL;

typedef enum {
    BTN_ID_TRIGGER = BUTTON_GPIO,
    BTN_ID_MENU = BUTTON_GPIO_2,
} button_id_t;

typedef struct {
    button_id_t id;
    TickType_t tick;
} button_event_t;

static void IRAM_ATTR button_isr_handler(void *arg) {
    button_id_t btn = (button_id_t)(int)arg;
    button_event_t evt = {
        .id = btn,
        .tick = xTaskGetTickCountFromISR(),
    };
    BaseType_t hp_task_woken = pdFALSE;
    if (s_button_queue) {
        xQueueSendFromISR(s_button_queue, &evt, &hp_task_woken);
    }
    if (hp_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static esp_lcd_panel_handle_t panel_handle = NULL;
static TickType_t s_last_ws_start_attempt = 0;

// --- Display ---
void app_main(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // GPIO Hold Dis
    if (PIN_NUM_BL >= 0) {
        gpio_hold_dis((gpio_num_t)PIN_NUM_BL);
        gpio_set_direction((gpio_num_t)PIN_NUM_BL, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PIN_NUM_BL, 0); // Keep backlight off until first clear
    }

    if (PIN_NUM_PWR >= 0) {
        gpio_set_direction((gpio_num_t)PIN_NUM_PWR, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PIN_NUM_PWR, 1);
    }

    if (PIN_NUM_RD >= 0) {
        gpio_set_direction((gpio_num_t)PIN_NUM_RD, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PIN_NUM_RD, 1);
    }

    // --- LCD Init ---
    ESP_LOGI(TAG, "Initialize I80 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t buscfg = {
        .dc_gpio_num = PIN_NUM_DC,
        .wr_gpio_num = PIN_NUM_WR,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {
            PIN_NUM_D0, PIN_NUM_D1, PIN_NUM_D2, PIN_NUM_D3,
            PIN_NUM_D4, PIN_NUM_D5, PIN_NUM_D6, PIN_NUM_D7,
            -1, -1, -1, -1, -1, -1, -1, -1
        },
        .bus_width = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * 2
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&buscfg, &i80_bus));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        // Keep queue depth at 1 so transfers finish before we reuse the same DMA buffer.
        .trans_queue_depth = 1,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, LCD_X_OFFSET, LCD_Y_OFFSET));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    if (PIN_NUM_BL >= 0) {
        gpio_set_level((gpio_num_t)PIN_NUM_BL, 1); // turn backlight on after init
    }

    // Backlight ON
    // Will be enabled after first explicit screen clear
    size_t out_buf_size = LCD_H_RES * LCD_V_RES * 2;
    uint8_t *out_buf = esp_lcd_i80_alloc_draw_buffer(io_handle, out_buf_size, MALLOC_CAP_SPIRAM);
    if (!out_buf) {
        out_buf = esp_lcd_i80_alloc_draw_buffer(io_handle, out_buf_size, MALLOC_CAP_INTERNAL);
    }
    if (out_buf) {
        render_status_screen(panel_handle, (uint16_t *)out_buf, "BOOTING", "Initializing display", 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
    } else {
        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        ESP_LOGE(TAG, "No decode buffer available; largest free block: %lu bytes", (unsigned long)largest);
    }

    battery_init();

    // --- Startup state machine ---
    startup_ctx_t startup_ctx;
    startup_init(&startup_ctx, (uint16_t *)out_buf, out_buf_size, panel_handle, WIFI_SSID, WIFI_PASS, STEP_DEBUG_PAUSE_MS, STATUS_ERROR_PAUSE_MS);

    // --- Button Init ---
    s_button_queue = xQueueCreate(10, sizeof(button_event_t));
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO) | (1ULL << BUTTON_GPIO_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Buttons are usually active low
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO, button_isr_handler, (void *)BUTTON_GPIO);
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO_2, button_isr_handler, (void *)BUTTON_GPIO_2);
    sleep_ctx_t sleep_ctx = {
        .button_gpio_1 = BUTTON_GPIO,
        .button_gpio_2 = BUTTON_GPIO_2,
        .pin_bl = PIN_NUM_BL,
        .pin_pwr = PIN_NUM_PWR,
        .lcd_x_offset = LCD_X_OFFSET,
        .lcd_y_offset = LCD_Y_OFFSET,
        .panel = panel_handle,
        .button_queue = s_button_queue,
        .last_ws_start_attempt = &s_last_ws_start_attempt,
        .wifi_connected = &s_wifi_connected,
    };

    if (!images_init()) {
        ESP_LOGE(TAG, "Images init failed");
    }
    image_slot_t *images = images_slots();
    int images_count = startup_ctx.images_count;
    int current_image = startup_ctx.current_image;

    // --- Loop ---
    TickType_t last_press_tick_trigger = 0;
    TickType_t last_press_tick_menu = 0;
    bool menu_visible = false;
    int menu_selected = 0;
    TickType_t last_switch_tick = xTaskGetTickCount();
    bool wifi_was_connected = s_wifi_connected;
    bool ws_was_connected = step_wss_is_connected();
    TickType_t last_wifi_retry_tick = xTaskGetTickCount();
    if (startup_ctx.step == START_STEP_READY && images_count > 0 && images[0].valid && out_buf) {
        if (images_fetch_and_show(&images[0], (uint16_t *)out_buf, out_buf_size, panel_handle, s_wifi_connected, step_wss_is_connected())) {
            current_image = (images_count > 1) ? 1 : 0;
            last_switch_tick = xTaskGetTickCount();
        } else if (!s_wifi_connected) {
            render_status_screen(panel_handle, (uint16_t *)out_buf, "Images", "Need Wi-Fi", 0x0000, 0xF800, s_wifi_connected, step_wss_is_connected());
            startup_ctx.step = START_STEP_DIGEST; // need Wi-Fi to download images
        }
    }
    while (1) {
        TickType_t now_tick = xTaskGetTickCount();
        bool wifi_state_changed = (s_wifi_connected != wifi_was_connected);
        bool wifi_just_connected = s_wifi_connected && !wifi_was_connected;
        bool ws_state_changed = (step_wss_is_connected() != ws_was_connected);
        wifi_was_connected = s_wifi_connected;
        ws_was_connected = step_wss_is_connected();

        bool driver_connected = wifi_ctrl_refresh_link();
        if (!driver_connected && (now_tick - last_wifi_retry_tick) >= pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL_MS)) {
            last_wifi_retry_tick = now_tick;
            ESP_LOGI(TAG, "Wi-Fi reconnect attempt...");
            esp_wifi_connect();
        }

        if (!s_wifi_connected && step_wss_has_client()) {
            ESP_LOGI(TAG, "Wi-Fi down; stopping WebSocket");
            step_wss_stop();
        }
        if (!step_wss_has_client() && s_wifi_connected &&
            startup_ctx.step == START_STEP_WSS) {
            if (out_buf) {
                render_status_screen(panel_handle, (uint16_t *)out_buf, "WebSocket", NULL, 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
            }
            vTaskDelay(pdMS_TO_TICKS(STEP_DEBUG_PAUSE_MS));
        }

        if (!step_wss_has_client() && s_wifi_connected &&
            (s_last_ws_start_attempt == 0 || (now_tick - s_last_ws_start_attempt) >= pdMS_TO_TICKS(WS_RETRY_INTERVAL_MS))) {
            s_last_ws_start_attempt = now_tick;
            step_wss_start();
        }

        startup_handle_loop(&startup_ctx, s_wifi_connected, wifi_state_changed, wifi_just_connected, ws_state_changed, menu_visible, images, (uint16_t *)out_buf, out_buf_size, panel_handle, &last_switch_tick, &current_image, STEP_DEBUG_PAUSE_MS, STATUS_ERROR_PAUSE_MS);

        images_count = startup_ctx.images_count;

        if ((wifi_state_changed || ws_state_changed) && out_buf) {
            icons_draw_status_badges((uint16_t *)out_buf, s_wifi_connected, step_wss_is_connected());
            render_panel_draw_bitmap_chunked(panel_handle, (uint16_t *)out_buf, LCD_H_RES, LCD_V_RES);
        }

        // Startup state machine handled by startup_handle_loop above

        button_event_t evt;
        while (xQueueReceive(s_button_queue, &evt, 0) == pdTRUE) {
            TickType_t *last_tick = (evt.id == BTN_ID_MENU) ? &last_press_tick_menu : &last_press_tick_trigger;
            if ((evt.tick - *last_tick) < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                continue;
            }
            *last_tick = evt.tick;

            if (evt.id == BTN_ID_MENU) {
                if (!out_buf) {
                    ESP_LOGE(TAG, "Menu buffer not available");
                    continue;
                }
                if (!menu_visible) {
                    menu_visible = true;
                    // Начинаем выбор сразу с FAST SLEEP (недоступные пункты пропускаем)
                    menu_selected = MENU_ITEM_FAST_SLEEP;
                    menu_render(panel_handle, (uint16_t *)out_buf, menu_selected, battery_read_mv(), s_wifi_connected, step_wss_is_connected());
                } else {
                    // Переключаемся только между FAST SLEEP и DEEP SLEEP; затем выходим из меню
                    if (menu_selected == MENU_ITEM_FAST_SLEEP) {
                        menu_selected = MENU_ITEM_DEEP_SLEEP;
                        menu_render(panel_handle, (uint16_t *)out_buf, menu_selected, battery_read_mv(), s_wifi_connected, step_wss_is_connected());
                    } else {
                        // Были на DEEP SLEEP — закрываем меню
                        menu_visible = false;
                        render_fill_color(panel_handle, (uint16_t *)out_buf, 0x0000);
                    }
                }
            } else if (evt.id == BTN_ID_TRIGGER) {
                if (menu_visible) {
                    if (menu_selected == MENU_ITEM_FAST_SLEEP) {
                        if (out_buf) {
                            render_status_screen(panel_handle, (uint16_t *)out_buf, "Fast sleep", "Press button to wake", 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
                        }
                        sleep_enter_fast(&sleep_ctx);
                        menu_visible = false;
                        if (out_buf) {
                            render_status_screen(panel_handle, (uint16_t *)out_buf, "Waking", "Restoring...", 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
                        }
                        last_switch_tick = xTaskGetTickCount();
                    } else if (menu_selected == MENU_ITEM_DEEP_SLEEP) {
                        if (out_buf) {
                            render_status_screen(panel_handle, (uint16_t *)out_buf, "Powering off", "Entering deep sleep", 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
                        }
                        sleep_enter_deep(&sleep_ctx);
                    }
                } else {
                    ESP_LOGI(TAG, "Button 0 Pressed! Sending Trigger...");
                    if (!s_wifi_connected) {
                        ESP_LOGW(TAG, "Trigger skipped: Wi-Fi not connected");
                        if (out_buf) {
                            render_status_screen(panel_handle, (uint16_t *)out_buf, "Trigger skipped", "No Wi-Fi connection", 0x0000, 0xF800, s_wifi_connected, step_wss_is_connected());
                            last_switch_tick = now_tick;
                        }
                        vTaskDelay(pdMS_TO_TICKS(STATUS_ERROR_PAUSE_MS));
                    } else {
                        if (out_buf) {
                            render_status_screen(panel_handle, (uint16_t *)out_buf, "Trigger", "Sending...", 0x0000, 0xFFFF, s_wifi_connected, step_wss_is_connected());
                        }
                        esp_err_t post_err = trigger_send();

                        if (post_err == ESP_OK) {
                            ESP_LOGI(TAG, "Trigger Sent.");
                            if (out_buf) {
                                render_status_screen(panel_handle, (uint16_t *)out_buf, "Trigger sent", "Success", 0x0000, 0x07E0, s_wifi_connected, step_wss_is_connected());
                                last_switch_tick = now_tick;
                            }
                        } else {
                            ESP_LOGE(TAG, "Trigger Failed: %s", esp_err_to_name(post_err));
                            if (out_buf) {
                                render_status_screen(panel_handle, (uint16_t *)out_buf, "Trigger failed", esp_err_to_name(post_err), 0x0000, 0xF800, s_wifi_connected, step_wss_is_connected());
                                last_switch_tick = now_tick;
                            }
                            vTaskDelay(pdMS_TO_TICKS(STATUS_ERROR_PAUSE_MS));
                        }
                    }
                }
            }
        }

        if (!menu_visible && images_count > 0 && startup_ctx.step == START_STEP_READY) {
            if ((now_tick - last_switch_tick) >= pdMS_TO_TICKS(IMAGE_ROTATION_INTERVAL_MS)) {
                last_switch_tick = now_tick;
                image_slot_t *img = &images[current_image];
                if (img->valid && out_buf) {
                    bool needs_download = !img->loaded;
                    if (needs_download && !s_wifi_connected) {
                        render_status_screen(panel_handle, (uint16_t *)out_buf, "Images", "Need Wi-Fi", 0x0000, 0xF800, s_wifi_connected, step_wss_is_connected());
                        startup_ctx.step = START_STEP_DIGEST; // block slideshow until Wi-Fi returns
                    } else if (!images_fetch_and_show(img, (uint16_t *)out_buf, out_buf_size, panel_handle, s_wifi_connected, step_wss_is_connected())) {
                        ESP_LOGE(TAG, "Failed to display image %d", current_image);
                    } else {
                        current_image = (current_image + 1) % images_count;
                    }
                }
            }
        } else if (menu_visible) {
            last_switch_tick = now_tick;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
