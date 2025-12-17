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
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// JPEG Decoder
#include "jpeg_decoder.h"

// --- CONFIG ---
#define WIFI_SSID      "AndroidAP4484"
#define WIFI_PASS      "dflg7101"
#define IMAGE_URL      "https://loremflickr.com/128/128"

// Note: Time Sync is handled in main.


// --- T-QT Pro Pinout ---
#define LCD_HOST    SPI2_HOST
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 2
#define PIN_NUM_CLK  3
#define PIN_NUM_CS   5
#define PIN_NUM_DC   6
#define PIN_NUM_RST  1
#define PIN_NUM_BL   10
#define LCD_H_RES 128
#define LCD_V_RES 128
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)

static const char *TAG = "NET-IMG";

// --- Wi-Fi ---
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_event_group;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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
#define MAX_JPEG_SIZE (20 * 1024)
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
    if (w > 128) w = 128; // Crop if necessary
    if (h > 128) h = 128;
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
    gpio_hold_dis((gpio_num_t)PIN_NUM_BL);

    // --- LCD Init ---
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2 + 1024
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 2, 1));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Backlight ON
    gpio_set_direction((gpio_num_t)PIN_NUM_BL, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_NUM_BL, 0);

    // --- Wifi ---
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    wifi_init_sta();
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Connected");

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

    // --- Buffers ---
    jpeg_buffer = heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpeg_buffer) jpeg_buffer = malloc(MAX_JPEG_SIZE); // Fallback to internal RAM

    // JPEG Output buffer
    // RGB565 = 2 bytes per pixel
    size_t out_buf_size = 128 * 128 * 2;
    uint8_t *out_buf = heap_caps_malloc(out_buf_size, MALLOC_CAP_DMA);
    if (!out_buf) out_buf = malloc(out_buf_size);

    // --- Button Init ---
    #define BUTTON_GPIO 0
    #define BUTTON_GPIO_2 47
    
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
    while (1) {
        // Button 0 (Boot) Check
        if (gpio_get_level(BUTTON_GPIO) == 0) {
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
            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce / prevent spam
        }

        // Periodic Image Fetch (every ~5 seconds)
        if (image_timer++ > 50) { // 50 * 100ms = 5000ms
            image_timer = 0;
            fetch_image();

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
                        .swap_color_bytes = 1,
                    }
                };
                esp_jpeg_image_output_t outimg;
                
                esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &outimg);
                if (res == ESP_OK) {
                    ESP_LOGI(TAG, "Decoded: %dx%d", outimg.width, outimg.height);
                    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, outimg.width, outimg.height, (uint16_t*)out_buf);
                } else {
                    ESP_LOGE(TAG, "JPEG Decode Error: %d", res);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check button every 100ms
    }
}
