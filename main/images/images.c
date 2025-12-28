#include "images.h"
#include "render/render.h"
#include "icons/icons.h"
#include "tls/certs.h"
#include "display_config.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include "jpeg_decoder.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "util/room_id.h"

static const char *TAG = "IMAGES";
static const char *API_URL = "https://volleycam.com/";

#define JSON_BUFFER_SIZE 65536
#define MAX_JPEG_SIZE (32 * 1024)

typedef struct {
    uint8_t *buf;
    int max_len;
    int *out_len;
} http_buffer_ctx_t;

static image_slot_t s_slots[MAX_IMAGES];
static char *s_json_buffer = NULL;
static char *s_jpeg_buffer = NULL;
static int s_jpeg_len = 0;
static int s_count = 0;

static void clear_slot_cache(void) {
    for (int i = 0; i < MAX_IMAGES; i++) {
        if (s_slots[i].jpeg) {
            free(s_slots[i].jpeg);
            s_slots[i].jpeg = NULL;
        }
        s_slots[i].valid = false;
        s_slots[i].loaded = false;
        s_slots[i].jpeg_len = 0;
        s_slots[i].url[0] = '\0';
    }
    s_count = 0;
}

bool images_init(void) {
    if (!s_jpeg_buffer) {
        s_jpeg_buffer = heap_caps_malloc(MAX_JPEG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_jpeg_buffer) {
            s_jpeg_buffer = malloc(MAX_JPEG_SIZE);
        }
    }
    if (!s_jpeg_buffer) {
        ESP_LOGE(TAG, "No decode buffer available");
        return false;
    }
    return true;
}

void images_clear_cache(void) {
    clear_slot_cache();
    if (s_json_buffer) {
        free(s_json_buffer);
        s_json_buffer = NULL;
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
    if (!s_json_buffer) {
        s_json_buffer = heap_caps_malloc(JSON_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_json_buffer) s_json_buffer = malloc(JSON_BUFFER_SIZE);
    }
    if (!s_json_buffer) {
        ESP_LOGE(TAG, "Failed to allocate JSON buffer");
        return false;
    }
    memset(s_json_buffer, 0, JSON_BUFFER_SIZE);
    int json_len = 0;
    http_buffer_ctx_t ctx = {
        .buf = (uint8_t *)s_json_buffer,
        .max_len = JSON_BUFFER_SIZE - 1,
        .out_len = &json_len,
    };
    char full_url[128];
    char room_id[7] = {0};
    if (!room_id_format(room_id, sizeof(room_id))) {
        memcpy(room_id, "000000", 6);
    }
    snprintf(full_url, sizeof(full_url), "%s?room=%s", API_URL, room_id);

    esp_http_client_config_t config = {
        .url = full_url,
        .event_handler = http_collect_event_handler,
        .user_data = &ctx,
        .timeout_ms = 10000,
        .buffer_size = 2048,
        .buffer_size_tx = 4096,
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = 0,
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
    return true;
}

static bool parse_thumbnail_urls(const char *json) {
    if (!json) return false;
    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse failed");
        return false;
    }

    // Helpers to collect URLs flexibly
    void add_url(const char *url, int *idx) {
        if (!url || !idx || *idx >= MAX_IMAGES) return;
        if (strncmp(url, "http", 4) != 0) return;
        strlcpy(s_slots[*idx].url, url, sizeof(s_slots[*idx].url));
        s_slots[*idx].valid = true;
        (*idx)++;
    }

    void extract_from_object(cJSON *obj, int *idx) {
        if (!cJSON_IsObject(obj) || !idx || *idx >= MAX_IMAGES) return;
        const char *keys[] = {"url", "thumbnail", "thumb", "thumb_url", "thumbnail_url"};
        for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
            cJSON *v = cJSON_GetObjectItem(obj, keys[i]);
            if (cJSON_IsString(v) && v->valuestring) {
                add_url(v->valuestring, idx);
                if (*idx >= MAX_IMAGES) return;
            }
        }
    }

    void process_array(cJSON *arr, int *idx) {
        if (!cJSON_IsArray(arr) || !idx || *idx >= MAX_IMAGES) return;
        cJSON *item = NULL;
        cJSON_ArrayForEach(item, arr) {
            if (*idx >= MAX_IMAGES) break;
            if (cJSON_IsString(item) && item->valuestring) {
                add_url(item->valuestring, idx);
            } else if (cJSON_IsObject(item)) {
                extract_from_object(item, idx);
            }
        }
    }

    clear_slot_cache();
    int idx = 0;
    // primary key
    cJSON *thumbs = cJSON_GetObjectItem(root, "thumbnails");
    if (cJSON_IsArray(thumbs)) {
        process_array(thumbs, &idx);
    }
    // other common arrays
    const char *array_keys[] = {"images", "data", "items"};
    for (size_t i = 0; i < sizeof(array_keys) / sizeof(array_keys[0]); i++) {
        if (idx >= MAX_IMAGES) break;
        cJSON *arr = cJSON_GetObjectItem(root, array_keys[i]);
        if (cJSON_IsArray(arr)) {
            process_array(arr, &idx);
        }
    }
    // root as array
    if (idx < MAX_IMAGES && cJSON_IsArray(root)) {
        process_array(root, &idx);
    }
    // fallback: scan first level
    if (idx == 0) {
        cJSON *child = NULL;
        cJSON_ArrayForEach(child, root) {
            if (idx >= MAX_IMAGES) break;
            if (cJSON_IsString(child) && child->valuestring) {
                add_url(child->valuestring, &idx);
            } else if (cJSON_IsObject(child)) {
                extract_from_object(child, &idx);
            } else if (cJSON_IsArray(child)) {
                process_array(child, &idx);
            }
        }
    }
    cJSON_Delete(root);
    s_count = idx;
    if (idx == 0) {
        int preview_len = strlen(json) > 256 ? 256 : (int)strlen(json);
        ESP_LOGW(TAG, "No valid thumbnail URLs found; JSON preview: %.*s", preview_len, json);
        return false;
    }
    ESP_LOGI(TAG, "Found %d thumbnails", idx);
    return true;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt) {
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

static bool fetch_jpeg_into_buffer(const char *url) {
    if (!url || !s_jpeg_buffer) return false;
    s_jpeg_len = 0;
    http_buffer_ctx_t ctx = {
        .buf = (uint8_t *)s_jpeg_buffer,
        .max_len = MAX_JPEG_SIZE,
        .out_len = &s_jpeg_len,
    };
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &ctx,
        .timeout_ms = 15000,
        .buffer_size = 2048,
        .buffer_size_tx = 8192,
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = 0,
        .keep_alive_enable = false,
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status >= 200 && status < 300) {
        ESP_LOGI(TAG, "JPEG fetched (%d bytes)", s_jpeg_len);
        return s_jpeg_len > 0;
    }
    ESP_LOGE(TAG, "JPEG fetch failed (%s), status=%d", esp_err_to_name(err), status);
    return false;
}

int images_fetch_digest(bool wifi_connected) {
    if (!wifi_connected) {
        ESP_LOGW(TAG, "Wi-Fi unavailable; cannot fetch digest");
        return 0;
    }
    if (!fetch_json_feed()) {
        return 0;
    }
    if (!parse_thumbnail_urls(s_json_buffer)) {
        return 0;
    }
    return s_count;
}

bool images_fetch_and_show(image_slot_t *slot,
                           uint16_t *decode_buf,
                           size_t decode_buf_size,
                           esp_lcd_panel_handle_t panel,
                           bool wifi_connected,
                           bool ws_connected) {
    if (!slot || !decode_buf) return false;
    if (!slot->loaded) {
        if (!wifi_connected) {
            ESP_LOGW(TAG, "Wi-Fi unavailable; cannot download image");
            return false;
        }
        ESP_LOGI(TAG, "Downloading thumbnail: %s", slot->url);
        if (!fetch_jpeg_into_buffer(slot->url)) {
            return false;
        }
        uint8_t *cpy = heap_caps_malloc(s_jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!cpy) cpy = malloc(s_jpeg_len);
        if (!cpy) {
            ESP_LOGE(TAG, "JPEG cache allocation failed");
            return false;
        }
        memcpy(cpy, s_jpeg_buffer, s_jpeg_len);
        slot->jpeg = cpy;
        slot->jpeg_len = s_jpeg_len;
        slot->loaded = true;
    }

    if (decode_buf_size >= (size_t)LCD_H_RES * (size_t)LCD_V_RES * 2) {
        render_framebuf_fill_color((uint16_t *)decode_buf, 0x0000);
    }

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = slot->jpeg,
        .indata_size = slot->jpeg_len,
        .outbuf = (uint8_t *)decode_buf,
        .outbuf_size = decode_buf_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {.swap_color_bytes = 0}
    };
    esp_jpeg_image_output_t outimg;
    esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "JPEG decode error: %d", res);
        return false;
    }

    icons_draw_status_badges((uint16_t *)decode_buf, wifi_connected, ws_connected);
    bool ok = render_panel_draw_bitmap_chunked(panel, (uint16_t *)decode_buf, outimg.width, outimg.height);
    if (ok) {
        ESP_LOGI(TAG, "Thumbnail shown: %dx%d", outimg.width, outimg.height);
    }
    return ok;
}

int images_get_count(void) {
    return s_count;
}

image_slot_t *images_slots(void) {
    return s_slots;
}
