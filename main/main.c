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
#include "esp_websocket_client.h"
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
#include "esp_attr.h"

// JPEG Decoder
#include "jpeg_decoder.h"

// --- CONFIG ---
#define WIFI_SSID      "AndroidAP4484"
#define WIFI_PASS      "dflg7101"
#define API_URL        "https://camera.boxhoster.com/"
#define WS_URL         "wss://camera.boxhoster.com/cable"
// ActionCable typically advertises both protocol names; send both to satisfy strict servers.
#define WS_SUBPROTO    "actioncable-v1-json,actioncable-unsupported"
#define WS_IDENTIFIER_RAW "{\"channel\":\"RecordingChannel\"}"
// This needs to be JSON-escaped inside the subscribe frame.
#define WS_IDENTIFIER_ESCAPED "{\\\"channel\\\":\\\"RecordingChannel\\\"}"
#define WS_SUBSCRIBE_MSG "{\"command\":\"subscribe\",\"identifier\":\"" WS_IDENTIFIER_ESCAPED "\"}"
#define WS_RETRY_INTERVAL_MS 5000
#define WS_BUFFER_SIZE 2048
#define WS_MESSAGE_BUF_SIZE 4096
#define WS_ORIGIN      "https://camera.boxhoster.com"

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
#define LCD_DMA_ALIGN 16
#define LCD_DMA_CHUNK_BYTES 4080

#define BUTTON_GPIO 0
#define BUTTON_GPIO_2 14
#define BUTTON_DEBOUNCE_MS 250
#define IMAGE_ROTATION_INTERVAL_MS 1000

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
#define JSON_BUFFER_SIZE 65536
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
static char *json_buffer = NULL; // allocated on demand; prefer PSRAM when available
static image_slot_t s_images[MAX_IMAGES];
static esp_lcd_panel_handle_t panel_handle = NULL;
// Two chunk buffers for ping-pong, so we never overwrite a buffer while a DMA transfer is still in flight.
static DMA_ATTR uint16_t s_chunk_buf[2][LCD_DMA_CHUNK_BYTES / sizeof(uint16_t)] __attribute__((aligned(LCD_DMA_ALIGN)));
static esp_websocket_client_handle_t s_ws_client = NULL;
static volatile bool s_feed_reload_requested = false;
static TickType_t s_last_ws_start_attempt = 0;
static char s_ws_msg_buf[WS_MESSAGE_BUF_SIZE];
static size_t s_ws_msg_len = 0;
static size_t s_ws_msg_expected = 0;

// Forward declarations for helpers used before definition
static void lcd_fill_color(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, uint16_t color);
static void framebuf_fill_color(uint16_t *frame_buf, uint16_t color);

static int calc_chunk_rows(int width_pixels) {
    size_t row_bytes = (size_t)width_pixels * sizeof(uint16_t);
    int rows = LCD_DMA_CHUNK_BYTES / row_bytes;
    if (rows < 1) {
        rows = 1;
    }
    while ((row_bytes * rows) % LCD_DMA_ALIGN != 0 && rows > 1) {
        rows--;
    }
    return rows;
}

static bool panel_draw_bitmap_chunked(const uint16_t *buf, int buf_width, int buf_height) {
    if (!panel_handle || !buf) {
        return false;
    }
    int buf_select = 0; // ping-pong between the two chunk buffers
    int draw_w = buf_width > LCD_H_RES ? LCD_H_RES : buf_width;
    int draw_h = buf_height > LCD_V_RES ? LCD_V_RES : buf_height;
    int rows_per_chunk = calc_chunk_rows(draw_w);
    if (rows_per_chunk < 1) {
        rows_per_chunk = 1;
    }
    int max_rows_in_buf = (int)(sizeof(s_chunk_buf) / sizeof(s_chunk_buf[0]) / draw_w);
    if (max_rows_in_buf < 1) {
        max_rows_in_buf = 1;
    }
    if (rows_per_chunk > max_rows_in_buf) {
        rows_per_chunk = max_rows_in_buf;
    }

    esp_err_t err = ESP_OK;
    for (int y = 0; y < draw_h && err == ESP_OK; y += rows_per_chunk) {
        int h = rows_per_chunk;
        if (y + h > draw_h) {
            h = draw_h - y;
        }
        const uint16_t *row = buf + (y * buf_width);
        for (int r = 0; r < h; r++) {
            memcpy(&s_chunk_buf[buf_select][r * draw_w], row + r * buf_width, draw_w * sizeof(uint16_t));
        }
        err = esp_lcd_panel_draw_bitmap(panel_handle, 0, y, draw_w, y + h, s_chunk_buf[buf_select]);
        buf_select ^= 1; // switch buffer for next chunk
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Draw bitmap failed: %s", esp_err_to_name(err));
    }
    return err == ESP_OK;
}

static bool allocate_json_buffer(void) {
    if (json_buffer) {
        return true;
    }
    json_buffer = heap_caps_malloc(JSON_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!json_buffer) {
        json_buffer = malloc(JSON_BUFFER_SIZE);
    }
    if (!json_buffer) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer (%d bytes)", JSON_BUFFER_SIZE);
        return false;
    }
    return true;
}

static void free_json_buffer(void) {
    if (json_buffer) {
        free(json_buffer);
        json_buffer = NULL;
    }
}

static void clear_image_cache(void) {
    for (int i = 0; i < MAX_IMAGES; i++) {
        if (s_images[i].jpeg) {
            free(s_images[i].jpeg);
        }
        memset(&s_images[i], 0, sizeof(image_slot_t));
    }
}

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
    if (!allocate_json_buffer()) {
        return false;
    }
    memset(json_buffer, 0, JSON_BUFFER_SIZE);
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
    if (json_len >= JSON_BUFFER_SIZE - 1) {
        ESP_LOGW(TAG, "JSON may be truncated (buffer %d bytes); consider increasing JSON_BUFFER_SIZE", JSON_BUFFER_SIZE);
    }
    ESP_LOGI(TAG, "JSON fetched (%d bytes)", json_len);
    return json_len > 0;
}

static bool parse_thumbnail_urls(const char *json, image_slot_t *slots, int *out_count) {
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
        memset(&slots[idx], 0, sizeof(image_slot_t));
        memcpy(slots[idx].url, url->valuestring, len);
        slots[idx].url[len] = '\0';
        slots[idx].valid = true;
        slots[idx].jpeg = NULL;
        slots[idx].jpeg_len = 0;
        slots[idx].loaded = false;
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

    // Clear working buffer only (do not push to panel) to avoid visible black flash between images.
    if (decode_buf_size >= (size_t)LCD_H_RES * (size_t)LCD_V_RES * 2) {
        framebuf_fill_color((uint16_t *)decode_buf, 0x0000);
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

    bool ok = panel_draw_bitmap_chunked((uint16_t *)decode_buf, outimg.width, outimg.height);
    if (ok) {
        ESP_LOGI(TAG, "Thumbnail shown: %dx%d", outimg.width, outimg.height);
    }
    return ok;
}

static int load_thumbnail_list(void) {
    if (!s_wifi_connected) {
        ESP_LOGW(TAG, "Skipping downloads; Wi-Fi not connected");
        return 0;
    }
    image_slot_t new_slots[MAX_IMAGES];
    memset(new_slots, 0, sizeof(new_slots));
    if (!fetch_json_feed()) {
        free_json_buffer();
        return 0;
    }
    int found = 0;
    if (!parse_thumbnail_urls(json_buffer, new_slots, &found)) {
        free_json_buffer();
        return 0;
    }
    free_json_buffer(); // release heap before allocating large decode buffers
    clear_image_cache();
    memcpy(s_images, new_slots, sizeof(new_slots));
    ESP_LOGI(TAG, "Cached %d thumbnail URLs", found);
    return found;
}

static void handle_actioncable_message(const char *payload, size_t len) {
    if (!payload || len == 0) {
        return;
    }
    // Inspect messages even if parsing later fails to confirm traffic on the wire.
    int preview_len = len > 256 ? 256 : (int)len;
    ESP_LOGI(TAG, "WS payload (%u bytes)%s: %.*s",
             (unsigned)len,
             len > 256 ? " [truncated]" : "",
             preview_len,
             payload);
    char *buf = malloc(len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "No heap for websocket message (%u bytes)", (unsigned)len);
        return;
    }
    memcpy(buf, payload, len);
    buf[len] = '\0';
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        ESP_LOGW(TAG, "WS JSON parse failed");
        return;
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (cJSON_IsString(type)) {
        if (strcmp(type->valuestring, "welcome") == 0 || strcmp(type->valuestring, "ping") == 0) {
            cJSON_Delete(root);
            return;
        }
        if (strcmp(type->valuestring, "confirm_subscription") == 0) {
            ESP_LOGI(TAG, "Subscribed to RecordingChannel (confirm_subscription)");
            cJSON_Delete(root);
            return;
        }
    }

    cJSON *message = cJSON_GetObjectItem(root, "message");
    if (cJSON_IsObject(message)) {
        cJSON *action = cJSON_GetObjectItem(message, "action");
        if (cJSON_IsString(action)) {
            ESP_LOGI(TAG, "WS message action: %s", action->valuestring);
        } else {
            ESP_LOGW(TAG, "WS message missing action field");
        }
        if (cJSON_IsString(action) && strcmp(action->valuestring, "upload_success") == 0) {
            ESP_LOGI(TAG, "Upload success notification received; scheduling feed reload");
            s_feed_reload_requested = true;
        }
    }
    cJSON_Delete(root);
}

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "WebSocket connected");
            const char *subscribe_msg = WS_SUBSCRIBE_MSG;
            ESP_LOGI(TAG, "Sending subscribe: %s", subscribe_msg);
            esp_websocket_client_send_text(data->client, subscribe_msg, strlen(subscribe_msg), pdMS_TO_TICKS(1000));
            break;
        }
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WebSocket disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            if (data->op_code == WS_TRANSPORT_OPCODES_TEXT && data->data_len > 0) {
                bool is_first = data->payload_offset == 0;
                bool is_last = (data->payload_offset + data->data_len) >= data->payload_len;
                if (is_first) {
                    s_ws_msg_len = 0;
                    s_ws_msg_expected = data->payload_len;
                }
                ESP_LOGI(TAG, "WS data chunk len=%d total=%d offset=%d first=%d last=%d",
                         data->data_len, data->payload_len, data->payload_offset, is_first, is_last);
                int log_len = data->data_len > 256 ? 256 : data->data_len;
                ESP_LOGI(TAG, "WS chunk preview: %.*s%s",
                         log_len, (const char *)data->data_ptr,
                         data->data_len > log_len ? " ..." : "");

                size_t remaining = WS_MESSAGE_BUF_SIZE - 1 - s_ws_msg_len;
                if ((size_t)data->data_len > remaining) {
                    ESP_LOGE(TAG, "WS message buffer overflow (need %u, have %u); dropping message",
                             (unsigned)data->data_len, (unsigned)remaining);
                    s_ws_msg_len = 0;
                    s_ws_msg_expected = 0;
                    break;
                }

                memcpy(s_ws_msg_buf + s_ws_msg_len, data->data_ptr, data->data_len);
                s_ws_msg_len += data->data_len;

                if (is_last || data->payload_len == 0) {
                    s_ws_msg_buf[s_ws_msg_len] = '\0';
                    ESP_LOGI(TAG, "WS assembled message len=%u expected=%u", (unsigned)s_ws_msg_len, (unsigned)s_ws_msg_expected);
                    handle_actioncable_message(s_ws_msg_buf, s_ws_msg_len);
                    s_ws_msg_len = 0;
                    s_ws_msg_expected = 0;
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGW(TAG, "WebSocket error encountered");
            break;
        default:
            break;
    }
}

static void start_recording_websocket(void) {
    if (s_ws_client) {
        return;
    }
    esp_websocket_client_config_t ws_cfg = {
        .uri = WS_URL,
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = sizeof(CLOUDFLARE_CA_PEM),
        .subprotocol = WS_SUBPROTO,
        .headers = "Origin: " WS_ORIGIN "\r\n",
        .buffer_size = WS_BUFFER_SIZE, // Allow larger handshake headers (e.g., via Cloudflare)
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 10000,
    };
    s_ws_client = esp_websocket_client_init(&ws_cfg);
    if (!s_ws_client) {
        ESP_LOGE(TAG, "Failed to init WebSocket client");
        return;
    }
    esp_websocket_register_events(s_ws_client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);
    ESP_LOGI(TAG, "Connecting to WebSocket: %s", WS_URL);
    esp_err_t err = esp_websocket_client_start(s_ws_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WebSocket start failed: %s", esp_err_to_name(err));
        esp_websocket_client_destroy(s_ws_client);
        s_ws_client = NULL;
    }
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
    panel_draw_bitmap_chunked(pixels, w, h);
}

static void lcd_fill_color(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, uint16_t color) {
    (void)panel;
    size_t pixel_count = (size_t)LCD_H_RES * (size_t)LCD_V_RES;
    for (size_t i = 0; i < pixel_count; i++) {
        frame_buf[i] = color;
    }
    panel_draw_bitmap_chunked(frame_buf, LCD_H_RES, LCD_V_RES);
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

static int lcd_text_width_px(const char *text) {
    if (!text) return 0;
    size_t len = strnlen(text, 64);
    return (int)len * (FONT_WIDTH + 1) * FONT_SCALE;
}

static void lcd_show_status_screen(uint16_t *frame_buf, const char *line1, const char *line2, uint16_t bg_color, uint16_t fg_color) {
    if (!frame_buf || !panel_handle) return;
    framebuf_fill_color(frame_buf, bg_color);
    int y = (LCD_V_RES / 2) - (FONT_HEIGHT * FONT_SCALE);
    if (line1) {
        int x = (LCD_H_RES - lcd_text_width_px(line1)) / 2;
        if (x < 0) x = 0;
        lcd_draw_text(frame_buf, x, y, line1, fg_color);
    }
    if (line2) {
        int x = (LCD_H_RES - lcd_text_width_px(line2)) / 2;
        if (x < 0) x = 0;
        lcd_draw_text(frame_buf, x, y + FONT_HEIGHT * FONT_SCALE + 10, line2, fg_color);
    }
    panel_draw_bitmap_chunked(frame_buf, LCD_H_RES, LCD_V_RES);
    if (PIN_NUM_BL >= 0) {
        gpio_set_level((gpio_num_t)PIN_NUM_BL, 1);
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
    (void)panel;
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
    panel_draw_bitmap_chunked(frame_buf, LCD_H_RES, LCD_V_RES);
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

    // Backlight ON
    // Will be enabled after first explicit screen clear
    size_t out_buf_size = LCD_H_RES * LCD_V_RES * 2;
    uint8_t *out_buf = esp_lcd_i80_alloc_draw_buffer(io_handle, out_buf_size, MALLOC_CAP_SPIRAM);
    if (!out_buf) {
        out_buf = esp_lcd_i80_alloc_draw_buffer(io_handle, out_buf_size, MALLOC_CAP_INTERNAL);
    }
    if (out_buf) {
        lcd_show_status_screen((uint16_t *)out_buf, "BOOTING", "Initializing display", 0x0000, 0xFFFF);
    } else {
        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        ESP_LOGE(TAG, "No decode buffer available; largest free block: %lu bytes", (unsigned long)largest);
    }

    init_battery_adc();

    // --- Wifi ---
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    if (out_buf) {
        lcd_show_status_screen((uint16_t *)out_buf, "Wi-Fi", "Connecting...", 0x0000, 0xFFFF);
    }
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
        if (out_buf) {
            lcd_show_status_screen((uint16_t *)out_buf, "Wi-Fi OK", WIFI_SSID, 0x0000, 0x07E0);
        }
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timeout, continuing without connection");
        if (out_buf) {
            lcd_show_status_screen((uint16_t *)out_buf, "Wi-Fi timeout", "Continuing offline", 0x0000, 0xF800);
        }
    }

    // --- SNTP Time Sync ---
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    if (out_buf) {
        lcd_show_status_screen((uint16_t *)out_buf, "Time sync", "Waiting for SNTP...", 0x0000, 0xFFFF);
    }

    int retry = 0;
    const int retry_count = 10; // limit blocking time during boot
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d) Status: %d", retry, retry_count, sntp_get_sync_status());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Time Set: %s", asctime(&timeinfo));
    if (out_buf) {
        if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            lcd_show_status_screen((uint16_t *)out_buf, "Time OK", "Continuing...", 0x0000, 0x07E0);
        } else {
            lcd_show_status_screen((uint16_t *)out_buf, "Time sync timeout", "TLS may fail", 0x0000, 0xF800);
        }
    }

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

    // Fetch JSON list before allocating large decode buffers to keep heap available for TLS
    if (out_buf) {
        if (s_wifi_connected) {
            lcd_show_status_screen((uint16_t *)out_buf, "Loading feed", "Fetching thumbnails...", 0x0000, 0xFFFF);
        } else {
            lcd_show_status_screen((uint16_t *)out_buf, "Offline mode", "No Wi-Fi connection", 0x0000, 0xFFFF);
        }
    }
    int images_count = load_thumbnail_list();
    int current_image = 0;

    jpeg_buffer = heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jpeg_buffer) jpeg_buffer = malloc(MAX_JPEG_SIZE); // Fallback to internal RAM

    if (!out_buf) {
        // Already logged above
    } else {
        lcd_show_status_screen((uint16_t *)out_buf, "Ready", "Starting slideshow", 0x0000, 0xFFFF);
    }

    // --- Loop ---
    TickType_t last_press_tick_trigger = 0;
    TickType_t last_press_tick_menu = 0;
    bool menu_visible = false;
    int menu_selected = 0;
    TickType_t last_switch_tick = xTaskGetTickCount();
    if (images_count > 0 && s_images[0].valid && out_buf) {
        fetch_and_draw_image(&s_images[0], out_buf, out_buf_size);
        current_image = (images_count > 1) ? 1 : 0;
        last_switch_tick = xTaskGetTickCount();
    }
    while (1) {
        TickType_t now_tick = xTaskGetTickCount();
        if (!s_ws_client && s_wifi_connected &&
            (s_last_ws_start_attempt == 0 || (now_tick - s_last_ws_start_attempt) >= pdMS_TO_TICKS(WS_RETRY_INTERVAL_MS))) {
            s_last_ws_start_attempt = now_tick;
            start_recording_websocket();
        }

        if (s_feed_reload_requested) {
            s_feed_reload_requested = false;
            ESP_LOGI(TAG, "Reloading feed after upload_success notification");
            if (out_buf) {
                lcd_show_status_screen((uint16_t *)out_buf, "Updating", "Loading latest photos", 0x0000, 0xFFFF);
            }
            int new_count = load_thumbnail_list();
            if (new_count > 0) {
                images_count = new_count;
                current_image = 0;
                if (!menu_visible && out_buf && s_images[0].valid) {
                    if (!fetch_and_draw_image(&s_images[0], out_buf, out_buf_size)) {
                        ESP_LOGE(TAG, "Failed to display refreshed image 0");
                    } else {
                        last_switch_tick = xTaskGetTickCount();
                        current_image = (images_count > 1) ? 1 : 0;
                    }
                }
            } else {
                ESP_LOGW(TAG, "Feed reload returned no images; keeping existing list");
            }
        }

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
                        if (out_buf) {
                            lcd_show_status_screen((uint16_t *)out_buf, "Powering off", "Entering deep sleep", 0x0000, 0xFFFF);
                        }
                        enter_deep_sleep();
                    }
                } else {
                    ESP_LOGI(TAG, "Button 0 Pressed! Sending Trigger...");
                    if (!s_wifi_connected) {
                        ESP_LOGW(TAG, "Trigger skipped: Wi-Fi not connected");
                        if (out_buf) {
                            lcd_show_status_screen((uint16_t *)out_buf, "Trigger skipped", "No Wi-Fi connection", 0x0000, 0xF800);
                            last_switch_tick = now_tick;
                        }
                    } else {
                        if (out_buf) {
                            lcd_show_status_screen((uint16_t *)out_buf, "Trigger", "Sending...", 0x0000, 0xFFFF);
                        }
                        esp_http_client_config_t config_post = {
                            .url = "https://camera.boxhoster.com/recorder/trigger",
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
                            if (out_buf) {
                                lcd_show_status_screen((uint16_t *)out_buf, "Trigger sent", "Success", 0x0000, 0x07E0);
                                last_switch_tick = now_tick;
                            }
                        } else {
                            ESP_LOGE(TAG, "Trigger Failed: %s", esp_err_to_name(post_err));
                            if (out_buf) {
                                lcd_show_status_screen((uint16_t *)out_buf, "Trigger failed", esp_err_to_name(post_err), 0x0000, 0xF800);
                                last_switch_tick = now_tick;
                            }
                        }
                    }
                }
            }
        }

        if (!menu_visible && images_count > 0) {
            if ((now_tick - last_switch_tick) >= pdMS_TO_TICKS(IMAGE_ROTATION_INTERVAL_MS)) {
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
