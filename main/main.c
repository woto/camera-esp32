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
#define API_URL        "https://coordination-zope-counter-newspapers.trycloudflare.com/"

static const char CLOUDFLARE_CA_PEM[] =
"-----BEGIN CERTIFICATE-----\n"
"MIIDtjCCA1ygAwIBAgIQCexmoDY34U8OA4t/L2F36jAKBggqhkjOPQQDAjA7MQsw\n"
"CQYDVQQGEwJVUzEeMBwGA1UEChMVR29vZ2xlIFRydXN0IFNlcnZpY2VzMQwwCgYD\n"
"VQQDEwNXRTEwHhcNMjUxMjE0MTc1NTM1WhcNMjYwMzE0MTg1NTE4WjAcMRowGAYD\n"
"VQQDExF0cnljbG91ZGZsYXJlLmNvbTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IA\n"
"BFHCYvEJFOEfTJDPyKGIexzkq5aJr58B3zjnciufKjZU+l+rwpvszaS99UoQjl+0\n"
"b+vDNi5MQ+RLpqhy6FTL4fSjggJfMIICWzAOBgNVHQ8BAf8EBAMCB4AwEwYDVR0l\n"
"BAwwCgYIKwYBBQUHAwEwDAYDVR0TAQH/BAIwADAdBgNVHQ4EFgQUolY55Q7XwP4Q\n"
"XWPZljNjSCwmaIcwHwYDVR0jBBgwFoAUkHeSNWfE/6jMqeZ72YB5e8yT+TgwXgYI\n"
"KwYBBQUHAQEEUjBQMCcGCCsGAQUFBzABhhtodHRwOi8vby5wa2kuZ29vZy9zL3dl\n"
"MS9DZXcwJQYIKwYBBQUHMAKGGWh0dHA6Ly9pLnBraS5nb29nL3dlMS5jcnQwMQYD\n"
"VR0RBCowKIIRdHJ5Y2xvdWRmbGFyZS5jb22CEyoudHJ5Y2xvdWRmbGFyZS5jb20w\n"
"EwYDVR0gBAwwCjAIBgZngQwBAgEwNgYDVR0fBC8wLTAroCmgJ4YlaHR0cDovL2Mu\n"
"cGtpLmdvb2cvd2UxLzBiMDdsS3lTZ1UwLmNybDCCAQQGCisGAQQB1nkCBAIEgfUE\n"
"gfIA8AB2AMs49xWJfIShRF9bwd37yW7ymlnNRwppBYWwyxTDFFjnAAABmx432nwA\n"
"AAQDAEcwRQIgWDmZTkbb0FsoAB/N9zdMeyIt/RiutNvkuic/RnQ8DJ4CIQCCkISz\n"
"uT8M87Je6gzH4N/2WvdbiV52RikykipPpLTjVgB2AA5XlLzzrqk+MxssmQez95Df\n"
"m8I9cTIl3SGpJaxhxU4hAAABmx432jIAAAQDAEcwRQIgMrW+b9zrfsOa6KXZqdvo\n"
"zdfq6+S7O11CxXPvagvgvdcCIQDr+xGOZpRDU0jdwlbaH+VBFdFxG6O+9oG0D9ks\n"
"t6rXTDAKBggqhkjOPQQDAgNIADBFAiEA+NibNwJov0iHz53ikUvVRVDr4KjtDAWN\n"
"DsHAp2+5+PkCICMUjEQMQXfxNQ07g0u7evuIC9hgB5dHqJrfxO70Fto8\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIICnzCCAiWgAwIBAgIQf/MZd5csIkp2FV0TttaF4zAKBggqhkjOPQQDAzBHMQsw\n"
"CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU\n"
"MBIGA1UEAxMLR1RTIFJvb3QgUjQwHhcNMjMxMjEzMDkwMDAwWhcNMjkwMjIwMTQw\n"
"MDAwWjA7MQswCQYDVQQGEwJVUzEeMBwGA1UEChMVR29vZ2xlIFRydXN0IFNlcnZp\n"
"Y2VzMQwwCgYDVQQDEwNXRTEwWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAARvzTr+\n"
"Z1dHTCEDhUDCR127WEcPQMFcF4XGGTfn1XzthkubgdnXGhOlCgP4mMTG6J7/EFmP\n"
"LCaY9eYmJbsPAvpWo4H+MIH7MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEDDAKBggr\n"
"BgEFBQcDAQYIKwYBBQUHAwIwEgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHQ4EFgQU\n"
"kHeSNWfE/6jMqeZ72YB5e8yT+TgwHwYDVR0jBBgwFoAUgEzW63T/STaj1dj8tT7F\n"
"avCUHYwwNAYIKwYBBQUHAQEEKDAmMCQGCCsGAQUFBzAChhhodHRwOi8vaS5wa2ku\n"
"Z29vZy9yNC5jcnQwKwYDVR0fBCQwIjAgoB6gHIYaaHR0cDovL2MucGtpLmdvb2cv\n"
"ci9yNC5jcmwwEwYDVR0gBAwwCjAIBgZngQwBAgEwCgYIKoZIzj0EAwMDaAAwZQIx\n"
"AOcCq1HW90OVznX+0RGU1cxAQXomvtgM8zItPZCuFQ8jSBJSjz5keROv9aYsAm5V\n"
"sQIwJonMaAFi54mrfhfoFNZEfuNMSQ6/bIBiNLiyoX46FohQvKeIoJ99cx7sUkFN\n"
"7uJW\n"
"-----END CERTIFICATE-----\n"
"-----BEGIN CERTIFICATE-----\n"
"MIIDejCCAmKgAwIBAgIQf+UwvzMTQ77dghYQST2KGzANBgkqhkiG9w0BAQsFADBX\n"
"MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE\n"
"CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIzMTEx\n"
"NTAzNDMyMVoXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT\n"
"GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFI0\n"
"MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAE83Rzp2iLYK5DuDXFgTB7S0md+8Fhzube\n"
"Rr1r1WEYNa5A3XP3iZEwWus87oV8okB2O6nGuEfYKueSkWpz6bFyOZ8pn6KY019e\n"
"WIZlD6GEZQbR3IvJx3PIjGov5cSr0R2Ko4H/MIH8MA4GA1UdDwEB/wQEAwIBhjAd\n"
"BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd\n"
"BgNVHQ4EFgQUgEzW63T/STaj1dj8tT7FavCUHYwwHwYDVR0jBBgwFoAUYHtmGkUN\n"
"l8qJUC99BM00qP/8/UswNgYIKwYBBQUHAQEEKjAoMCYGCCsGAQUFBzAChhpodHRw\n"
"Oi8vaS5wa2kuZ29vZy9nc3IxLmNydDAtBgNVHR8EJjAkMCKgIKAehhxodHRwOi8v\n"
"Yy5wa2kuZ29vZy9yL2dzcjEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqG\n"
"SIb3DQEBCwUAA4IBAQAYQrsPBtYDh5bjP2OBDwmkoWhIDDkic574y04tfzHpn+cJ\n"
"odI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1hY\n"
"+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/Cs\n"
"kGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbep\n"
"8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K1\n"
"vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl\n"
"-----END CERTIFICATE-----\n";

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
#define BUTTON_DEBOUNCE_MS 250

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
static QueueHandle_t s_button_queue = NULL;

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_cali_enabled = false;

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
    // Keep Wi-Fi awake during transfers to reduce TLS timeouts
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// --- HTTP & Image ---
#define MAX_JPEG_SIZE (32 * 1024)
#define JSON_BUFFER_SIZE 4096
#define MAX_IMAGES 32
#define MAX_URL_LEN 256

typedef struct {
    uint8_t *buf;
    int max_len;
    int *out_len;
} http_buffer_ctx_t;

typedef struct {
    char url[MAX_URL_LEN];
    uint8_t *jpeg;
    int jpeg_len;
    bool valid;
    bool loaded;
} image_slot_t;

static char *jpeg_buffer = NULL;
static int jpeg_len = 0;
static char json_buffer[JSON_BUFFER_SIZE];
static image_slot_t s_images[MAX_IMAGES];
static esp_lcd_panel_handle_t panel_handle = NULL;

static esp_err_t http_collect_event_handler(esp_http_client_event_t *evt) {
    http_buffer_ctx_t *ctx = (http_buffer_ctx_t *)evt->user_data;
    if (!ctx || !ctx->buf || !ctx->out_len) return ESP_OK;
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        int remaining = ctx->max_len - *(ctx->out_len);
        int to_copy = evt->data_len < remaining ? evt->data_len : remaining;
        if (to_copy > 0) {
            memcpy(ctx->buf + *(ctx->out_len), evt->data, to_copy);
            *(ctx->out_len) += to_copy;
        }
    }
    return ESP_OK;
}

static bool fetch_json_feed(void) {
    memset(json_buffer, 0, sizeof(json_buffer));
    int json_len = 0;
    http_buffer_ctx_t ctx = {
        .buf = (uint8_t *)json_buffer,
        .max_len = JSON_BUFFER_SIZE - 1,
        .out_len = &json_len,
    };
    esp_http_client_config_t config = {
        .url = API_URL,
        .event_handler = http_collect_event_handler,
        .user_data = &ctx,
        .timeout_ms = 10000,
        .buffer_size = 2048,
        .buffer_size_tx = 4096, // long signed URLs can exceed default tx buffer
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = sizeof(CLOUDFLARE_CA_PEM),
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_err_t err = esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "JSON fetch failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "JSON fetched (%d bytes)", json_len);
    return json_len > 0;
}

static bool parse_thumbnail_urls(const char *json, int *out_count) {
    if (!json || !out_count) return false;
    *out_count = 0;

    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error");
        return false;
    }

    cJSON *thumbs = cJSON_GetObjectItem(root, "thumbnails");
    if (!cJSON_IsArray(thumbs)) {
        ESP_LOGW(TAG, "No thumbnails array in JSON");
        cJSON_Delete(root);
        return false;
    }

    int idx = 0;
    cJSON *thumb = NULL;
    cJSON_ArrayForEach(thumb, thumbs) {
        if (idx >= MAX_IMAGES) break;
        cJSON *url = cJSON_GetObjectItem(thumb, "url");
        if (!cJSON_IsString(url) || !url->valuestring) {
            continue;
        }
        size_t len = strnlen(url->valuestring, MAX_URL_LEN - 1);
        memset(&s_images[idx], 0, sizeof(image_slot_t));
        memcpy(s_images[idx].url, url->valuestring, len);
        s_images[idx].url[len] = '\0';
        s_images[idx].valid = true;
        s_images[idx].jpeg = NULL;
        s_images[idx].jpeg_len = 0;
        s_images[idx].loaded = false;
        idx++;
    }

    cJSON_Delete(root);
    *out_count = idx;
    if (idx == 0) {
        ESP_LOGW(TAG, "No valid thumbnail URLs found");
        return false;
    }
    ESP_LOGI(TAG, "Found %d thumbnails", idx);
    return true;
}

static bool fetch_jpeg_into_buffer(const char *url) {
    if (!url || !jpeg_buffer) return false;
    jpeg_len = 0;
    http_buffer_ctx_t ctx = {
        .buf = (uint8_t *)jpeg_buffer,
        .max_len = MAX_JPEG_SIZE,
        .out_len = &jpeg_len,
    };
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_collect_event_handler,
        .user_data = &ctx,
        .timeout_ms = 15000,
        .buffer_size = 2048,
        .buffer_size_tx = 8192, // handle very long signed URLs
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = sizeof(CLOUDFLARE_CA_PEM),
        .keep_alive_enable = false,
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status >= 200 && status < 300) {
        ESP_LOGI(TAG, "JPEG fetched (%d bytes)", jpeg_len);
        return jpeg_len > 0;
    }
    ESP_LOGE(TAG, "JPEG fetch failed (%s): %s, status=%d", url, esp_err_to_name(err), status);
    return false;
}

static bool fetch_and_draw_image(image_slot_t *slot, uint8_t *decode_buf, size_t decode_buf_size) {
    if (!slot || !slot->valid || !decode_buf) return false;

    // Download once and cache JPEG
    if (!slot->loaded || !slot->jpeg) {
        ESP_LOGI(TAG, "Downloading thumbnail: %s", slot->url);
        if (!fetch_jpeg_into_buffer(slot->url)) {
            return false;
        }
        uint8_t *cpy = heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!cpy) cpy = malloc(jpeg_len);
        if (!cpy) {
            ESP_LOGE(TAG, "JPEG cache allocation failed");
            return false;
        }
        memcpy(cpy, jpeg_buffer, jpeg_len);
        slot->jpeg = cpy;
        slot->jpeg_len = jpeg_len;
        slot->loaded = true;
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = slot->jpeg,
        .indata_size = slot->jpeg_len,
        .outbuf = decode_buf,
        .outbuf_size = decode_buf_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 0,
        }
    };
    esp_jpeg_image_output_t outimg;
    esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode error: %d", res);
        return false;
    }

    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, outimg.width, outimg.height, decode_buf);
    ESP_LOGI(TAG, "Thumbnail shown: %dx%d", outimg.width, outimg.height);
    return true;
}

static int load_thumbnail_list(void) {
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "Skipping downloads; Wi-Fi not connected");
        return 0;
    }
    if (!fetch_json_feed()) {
        return 0;
    }
    int found = 0;
    if (!parse_thumbnail_urls(json_buffer, &found)) {
        return 0;
    }
    ESP_LOGI(TAG, "Cached %d thumbnail URLs", found);
    return found;
}

// --- Display ---
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
    uint8_t *out_buf = heap_caps_malloc(out_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out_buf) out_buf = heap_caps_malloc(out_buf_size, MALLOC_CAP_DMA);
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

    // --- Loop ---
    TickType_t last_press_tick_trigger = 0;
    TickType_t last_press_tick_menu = 0;
    bool menu_visible = false;
    int menu_selected = 0;
    int images_count = 0;
    if (out_buf) {
        images_count = load_thumbnail_list();
    } else {
        ESP_LOGE(TAG, "No decode buffer available; cannot download thumbnails");
    }
    int current_image = 0;
    TickType_t last_switch_tick = xTaskGetTickCount();
    if (images_count > 0 && s_images[0].valid && out_buf) {
        fetch_and_draw_image(&s_images[0], out_buf, out_buf_size);
        current_image = (images_count > 1) ? 1 : 0;
        last_switch_tick = xTaskGetTickCount();
    }
    while (1) {
        TickType_t now_tick = xTaskGetTickCount();
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
            } else if (evt.id == BTN_ID_TRIGGER) {
                if (menu_visible) {
                    if (menu_selected == MENU_ITEM_POWER) {
                        enter_deep_sleep();
                    }
                } else {
                    ESP_LOGI(TAG, "Button 0 Pressed! Sending Trigger...");

                    esp_http_client_config_t config_post = {
                        .url = "https://coordination-zope-counter-newspapers.trycloudflare.com/recorder/trigger",
                        .method = HTTP_METHOD_POST,
                        .timeout_ms = 10000,
                        .cert_pem = CLOUDFLARE_CA_PEM,
                        .cert_len = sizeof(CLOUDFLARE_CA_PEM),
                    };
                    esp_http_client_handle_t client_post = esp_http_client_init(&config_post);
                    esp_err_t post_err = esp_http_client_perform(client_post);
                    esp_http_client_cleanup(client_post);

                    if (post_err == ESP_OK) {
                        ESP_LOGI(TAG, "Trigger Sent.");
                    } else {
                        ESP_LOGE(TAG, "Trigger Failed: %s", esp_err_to_name(post_err));
                    }
                }
            }
        }

        if (!menu_visible && images_count > 0) {
            if ((now_tick - last_switch_tick) >= pdMS_TO_TICKS(5000)) {
                last_switch_tick = now_tick;
                image_slot_t *img = &s_images[current_image];
                if (img->valid && out_buf) {
                    if (!fetch_and_draw_image(img, out_buf, out_buf_size)) {
                        ESP_LOGE(TAG, "Failed to display image %d", current_image);
                    }
                    current_image = (current_image + 1) % images_count;
                }
            }
        } else if (menu_visible) {
            last_switch_tick = now_tick;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
