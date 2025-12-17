#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_sntp.h"
#include "driver/gpio.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// JPEG Decoder
#include "jpeg_decoder.h"

// --- CONFIG ---
#define WIFI_SSID      "AndroidAP4484"
#define WIFI_PASS      "dflg7101"
#define IMAGE_URL      "https://loremflickr.com/170/320"

// Note: Time Sync is handled in main.


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
#define LCD_H_RES 170
#define LCD_V_RES 320
#define LCD_X_OFFSET 35
#define LCD_Y_OFFSET 0
#define LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)

#define BUTTON_GPIO 0
#define BUTTON_GPIO_2 14

#define BAT_ADC_UNIT ADC_UNIT_1
#define BAT_ADC_CHANNEL ADC_CHANNEL_3
#define BAT_ADC_ATTEN ADC_ATTEN_DB_12
#define BAT_VOLT_DIVIDER_NUM 2
#define BAT_VOLT_DIVIDER_DEN 1

static const char *TAG = "NET-IMG";

// --- Wi-Fi ---
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;
static volatile bool s_wifi_connected = false;

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_cali_enabled = false;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// --- HTTP & Image ---

// --- HTTP & Image ---
#define MAX_JPEG_SIZE (80 * 1024)
static char *jpeg_buffer = NULL;
static int jpeg_len = 0;

esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (jpeg_len + evt->data_len < MAX_JPEG_SIZE) {
                memcpy(jpeg_buffer + jpeg_len, evt->data, evt->data_len);
                jpeg_len += evt->data_len;
            }
            break;
        default: break;
    }
    return ESP_OK;
}

void fetch_image() {
    ESP_LOGI(TAG, "Fetching image...");
    jpeg_len = 0;
    
    esp_http_client_config_t config = {
        .url = IMAGE_URL,
        .event_handler = _http_event_handler,
        .timeout_ms = 10000,
        .buffer_size = 4096,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP Local Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                (int)esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

// --- Display ---
esp_lcd_panel_handle_t panel_handle = NULL;

void draw_rgb565(uint16_t *pixels, int w, int h) {
    if (w > LCD_H_RES) w = LCD_H_RES; // Crop if necessary
    if (h > LCD_V_RES) h = LCD_V_RES;
    // Swap bytes for ESP LCD if needed or rely on driver.
    // esp_jpeg_dec usually outputs Big Endian RGB565? No, it's usually host order.
    // ST7789 expects Big Endian.
    // If output is Little Endian, we swap.
    // Let's assume we need to swap for now as standard C arrays are LE on ESP32.
    for (int i=0; i<w*h; i++) {
        pixels[i] = (pixels[i] >> 8) | (pixels[i] << 8);
    }
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, w, h, pixels);
}

static void lcd_fill_color(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, uint16_t color) {
    size_t pixel_count = (size_t)LCD_H_RES * (size_t)LCD_V_RES;
    for (size_t i = 0; i < pixel_count; i++) {
        frame_buf[i] = color;
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, frame_buf);
}

#define FONT_FIRST_CHAR 32
#define FONT_LAST_CHAR 126
#define FONT_WIDTH 5
#define FONT_HEIGHT 7
#define FONT_SCALE 2

static const uint8_t font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5f, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7f, 0x14, 0x7f, 0x14},
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1c, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1c, 0x00},
    {0x14, 0x08, 0x3e, 0x08, 0x14}, {0x08, 0x08, 0x3e, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
    {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3e}, {0x7e, 0x11, 0x11, 0x11, 0x7e},
    {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41},
    {0x7f, 0x09, 0x09, 0x09, 0x01}, {0x3e, 0x41, 0x49, 0x49, 0x7a},
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41},
    {0x7f, 0x40, 0x40, 0x40, 0x40}, {0x7f, 0x02, 0x0c, 0x02, 0x7f},
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
    {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e},
    {0x7f, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7f, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7f, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04}, {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00}, {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7f, 0x48, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7f}, {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7e, 0x09, 0x01, 0x02}, {0x0c, 0x52, 0x52, 0x52, 0x3e},
    {0x7f, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7d, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3d, 0x00}, {0x7f, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7f, 0x40, 0x00}, {0x7c, 0x04, 0x18, 0x04, 0x78},
    {0x7c, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7c, 0x14, 0x14, 0x14, 0x08}, {0x08, 0x14, 0x14, 0x18, 0x7c},
    {0x7c, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3f, 0x44, 0x40, 0x20}, {0x3c, 0x40, 0x40, 0x20, 0x7c},
    {0x1c, 0x20, 0x40, 0x20, 0x1c}, {0x3c, 0x40, 0x30, 0x40, 0x3c},
    {0x44, 0x28, 0x10, 0x28, 0x44}, {0x0c, 0x50, 0x50, 0x50, 0x3c},
    {0x44, 0x64, 0x54, 0x4c, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7f, 0x00, 0x00}, {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x08, 0x04, 0x08, 0x10, 0x08}
};

static void framebuf_fill_color(uint16_t *frame_buf, uint16_t color) {
    size_t pixel_count = (size_t)LCD_H_RES * (size_t)LCD_V_RES;
    for (size_t i = 0; i < pixel_count; i++) {
        frame_buf[i] = color;
    }
}

static void lcd_draw_char(uint16_t *frame_buf, int x, int y, char c, uint16_t color) {
    if (!frame_buf) return;
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
        c = '?';
    }
    const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
    for (int col = 0; col < FONT_WIDTH; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < FONT_HEIGHT; row++) {
            if (line & 0x01) {
                int px = x + col * FONT_SCALE;
                int py = y + row * FONT_SCALE;
                for (int dy = 0; dy < FONT_SCALE; dy++) {
                    for (int dx = 0; dx < FONT_SCALE; dx++) {
                        int fx = px + dx;
                        int fy = py + dy;
                        if (fx >= 0 && fy >= 0 && fx < LCD_H_RES && fy < LCD_V_RES) {
                            frame_buf[fy * LCD_H_RES + fx] = color;
                        }
                    }
                }
            }
            line >>= 1;
        }
    }
}

static void lcd_draw_text(uint16_t *frame_buf, int x, int y, const char *text, uint16_t color) {
    if (!frame_buf || !text) return;
    int cursor_x = x;
    while (*text) {
        lcd_draw_char(frame_buf, cursor_x, y, *text, color);
        cursor_x += (FONT_WIDTH + 1) * FONT_SCALE;
        text++;
    }
}

#define MENU_ITEMS 4
#define MENU_ITEM_POWER 3

static void lcd_fill_rect(uint16_t *frame_buf, int x, int y, int w, int h, uint16_t color) {
    if (!frame_buf) return;
    if (x < 0 || y < 0 || x + w > LCD_H_RES || y + h > LCD_V_RES) return;
    for (int yy = y; yy < y + h; yy++) {
        uint16_t *row = frame_buf + (yy * LCD_H_RES) + x;
        for (int xx = 0; xx < w; xx++) {
            row[xx] = color;
        }
    }
}

static void lcd_draw_rect_outline(uint16_t *frame_buf, int x, int y, int w, int h, uint16_t color) {
    if (!frame_buf) return;
    if (x < 0 || y < 0 || x + w > LCD_H_RES || y + h > LCD_V_RES) return;
    lcd_fill_rect(frame_buf, x, y, w, 1, color);
    lcd_fill_rect(frame_buf, x, y + h - 1, w, 1, color);
    lcd_fill_rect(frame_buf, x, y, 1, h, color);
    lcd_fill_rect(frame_buf, x + w - 1, y, 1, h, color);
}

static void lcd_render_menu(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, int selected, int bat_mv) {
    if (!frame_buf) return;
    framebuf_fill_color(frame_buf, 0x0000);
    const int box_w = LCD_H_RES - 30;
    const int box_h = 40;
    const int gap = 10;
    const int total_h = MENU_ITEMS * box_h + (MENU_ITEMS - 1) * gap;
    const int start_y = (LCD_V_RES - total_h) / 2;
    const int start_x = 15;
    char voltage_label[24];
    if (bat_mv > 0) {
        snprintf(voltage_label, sizeof(voltage_label), "VOLT %d.%02dV", bat_mv / 1000, (bat_mv % 1000) / 10);
    } else {
        snprintf(voltage_label, sizeof(voltage_label), "VOLT --.-V");
    }
    const char *labels[MENU_ITEMS] = {
        voltage_label,
        "EMPTY",
        "EMPTY",
        "POWER OFF",
    };
    for (int i = 0; i < MENU_ITEMS; i++) {
        int y = start_y + i * (box_h + gap);
        if (i == selected) {
            lcd_fill_rect(frame_buf, start_x, y, box_w, box_h, 0xFFFF);
            lcd_draw_rect_outline(frame_buf, start_x, y, box_w, box_h, 0x0000);
            lcd_draw_text(frame_buf, start_x + 10, y + 12, labels[i], 0x0000);
        } else {
            lcd_draw_rect_outline(frame_buf, start_x, y, box_w, box_h, 0xFFFF);
            lcd_draw_text(frame_buf, start_x + 10, y + 12, labels[i], 0xFFFF);
        }
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_H_RES, LCD_V_RES, frame_buf);
}

static void enter_deep_sleep(void) {
    uint64_t mask = 0;
    if (esp_sleep_is_valid_wakeup_gpio(BUTTON_GPIO)) {
        rtc_gpio_init(BUTTON_GPIO);
        rtc_gpio_set_direction(BUTTON_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(BUTTON_GPIO);
        rtc_gpio_pulldown_dis(BUTTON_GPIO);
        mask |= (1ULL << BUTTON_GPIO);
    }
    if (esp_sleep_is_valid_wakeup_gpio(BUTTON_GPIO_2)) {
        rtc_gpio_init(BUTTON_GPIO_2);
        rtc_gpio_set_direction(BUTTON_GPIO_2, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pullup_en(BUTTON_GPIO_2);
        rtc_gpio_pulldown_dis(BUTTON_GPIO_2);
        mask |= (1ULL << BUTTON_GPIO_2);
    }
    if (mask == 0) {
        ESP_LOGE(TAG, "No valid wakeup GPIOs configured, aborting sleep");
        return;
    }
    while (gpio_get_level(BUTTON_GPIO) == 0 || gpio_get_level(BUTTON_GPIO_2) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW));
    if (PIN_NUM_BL >= 0) {
        gpio_set_level((gpio_num_t)PIN_NUM_BL, 0);
    }
    if (PIN_NUM_PWR >= 0) {
        gpio_set_level((gpio_num_t)PIN_NUM_PWR, 0);
    }
    esp_deep_sleep_start();
}

static void init_battery_adc(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    if (adc_oneshot_new_unit(&init_cfg, &s_adc_handle) != ESP_OK) {
        s_adc_handle = NULL;
        return;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = BAT_ADC_ATTEN
    };
    if (adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg) != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = BAT_ADC_UNIT,
            .atten = BAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
            s_adc_cali_enabled = true;
        }
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = BAT_ADC_UNIT,
            .atten = BAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
            s_adc_cali_enabled = true;
        }
    }
#endif
}

static int read_battery_voltage_mv(void) {
    if (!s_adc_handle) {
        return -1;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return -1;
    }
    if (s_adc_cali_enabled) {
        int voltage_mv = 0;
        if (adc_cali_raw_to_voltage(s_adc_cali, raw, &voltage_mv) == ESP_OK) {
            return (voltage_mv * BAT_VOLT_DIVIDER_NUM) / BAT_VOLT_DIVIDER_DEN;
        }
    }
    return ((raw * 1100) / 4095) * BAT_VOLT_DIVIDER_NUM / BAT_VOLT_DIVIDER_DEN;
}

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
        .trans_queue_depth = 10,
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

    // Backlight ON
    if (PIN_NUM_BL >= 0) {
        gpio_set_direction((gpio_num_t)PIN_NUM_BL, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PIN_NUM_BL, 1);
    }

    init_battery_adc();

    // --- Buffers ---
    jpeg_buffer = heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpeg_buffer) jpeg_buffer = malloc(MAX_JPEG_SIZE); // Fallback to internal RAM

    // JPEG Output buffer
    // RGB565 = 2 bytes per pixel
    size_t out_buf_size = LCD_H_RES * LCD_V_RES * 2;
    uint8_t *out_buf = heap_caps_malloc(out_buf_size, MALLOC_CAP_DMA);
    if (!out_buf) out_buf = malloc(out_buf_size);

    if (out_buf) {
        lcd_fill_color(panel_handle, (uint16_t *)out_buf, 0xFFFF);
        vTaskDelay(pdMS_TO_TICKS(200));
        lcd_fill_color(panel_handle, (uint16_t *)out_buf, 0x0000);
    }

    // --- Wifi ---
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    wifi_init_sta();
    EventBits_t wifi_bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(10000)
    );
    if (wifi_bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi Connected");
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timeout, continuing without connection");
    }

    // --- SNTP Time Sync ---
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    int retry = 0;
    const int retry_count = 20;
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d) Status: %d", retry, retry_count, sntp_get_sync_status());
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Time Set: %s", asctime(&timeinfo));

    // --- Button Init ---
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO) | (1ULL << BUTTON_GPIO_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Buttons are usually active low
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    // --- Loop ---
    int image_timer = 0;
    bool menu_visible = false;
    int menu_selected = 0;
    int last_btn_trigger = 1;
    int last_btn_menu = 1;
    TickType_t last_press_trigger = 0;
    TickType_t last_press_menu = 0;
    while (1) {
        int btn_trigger = gpio_get_level(BUTTON_GPIO);
        int btn_menu = gpio_get_level(BUTTON_GPIO_2);
        TickType_t now_tick = xTaskGetTickCount();
        if (last_btn_menu == 1 && btn_menu == 0 && (now_tick - last_press_menu) > pdMS_TO_TICKS(250)) {
            last_press_menu = now_tick;
            if (!menu_visible) {
                menu_visible = true;
                menu_selected = 0;
                lcd_render_menu(panel_handle, (uint16_t *)out_buf, menu_selected, read_battery_voltage_mv());
            } else {
                menu_selected++;
                if (menu_selected >= MENU_ITEMS) {
                    menu_visible = false;
                    lcd_fill_color(panel_handle, (uint16_t *)out_buf, 0x0000);
                } else {
                    lcd_render_menu(panel_handle, (uint16_t *)out_buf, menu_selected, read_battery_voltage_mv());
                }
            }
        }

        if (last_btn_trigger == 1 && btn_trigger == 0 && (now_tick - last_press_trigger) > pdMS_TO_TICKS(250)) {
            last_press_trigger = now_tick;
            if (menu_visible) {
                if (menu_selected == MENU_ITEM_POWER) {
                    enter_deep_sleep();
                }
            } else {
                ESP_LOGI(TAG, "Button 0 Pressed! Sending Trigger...");

                esp_http_client_config_t config_post = {
                    .url = "http://chemical-topics-steady-morris.trycloudflare.com/recorder/trigger",
                    .method = HTTP_METHOD_POST,
                    .timeout_ms = 10000,
                    .crt_bundle_attach = esp_crt_bundle_attach,
                };
                esp_http_client_handle_t client_post = esp_http_client_init(&config_post);
                esp_http_client_perform(client_post);
                esp_http_client_cleanup(client_post);

                ESP_LOGI(TAG, "Trigger Sent.");
            }
        }
        last_btn_trigger = btn_trigger;
        last_btn_menu = btn_menu;

        // Periodic Image Fetch (every ~5 seconds)
        if (!menu_visible && image_timer++ > 50) { // 50 * 100ms = 5000ms
            image_timer = 0;
            if (s_wifi_connected) {
                fetch_image();
            } else {
                ESP_LOGW(TAG, "Skipping fetch; Wi-Fi not connected");
            }

            if (jpeg_len > 0) {
                ESP_LOGI(TAG, "Decoding JPEG (%d bytes)...", jpeg_len);
                
                esp_jpeg_image_cfg_t jpeg_cfg = {
                    .indata = (uint8_t*)jpeg_buffer,
                    .indata_size = jpeg_len,
                    .outbuf = out_buf,
                    .outbuf_size = out_buf_size,
                    .out_format = JPEG_IMAGE_FORMAT_RGB565,
                    .out_scale = JPEG_IMAGE_SCALE_0,
                    .flags = {
                        .swap_color_bytes = 0,
                    }
                };
                esp_jpeg_image_output_t outimg;
                
                esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &outimg);
                if (res == ESP_OK) {
                    ESP_LOGI(TAG, "Decoded: %dx%d", outimg.width, outimg.height);
                    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, outimg.width, outimg.height, (uint16_t*)out_buf);
                } else {
                    ESP_LOGE(TAG, "JPEG Decode Error: %d", res);
                    if (out_buf) {
                        lcd_fill_color(panel_handle, (uint16_t *)out_buf, 0xFFFF);
                    }
                }
            } else if (out_buf && s_wifi_connected) {
                lcd_fill_color(panel_handle, (uint16_t *)out_buf, 0xFFFF);
            }
        } else if (menu_visible) {
            image_timer = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check button every 100ms
    }
}
