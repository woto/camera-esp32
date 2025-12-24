#include "trigger.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "tls/certs.h"
#include "util/room_id.h"

static const char *TAG = "TRIGGER";

esp_err_t trigger_send(void) {
    // Получаем короткий ID комнаты: суффикс MAC (младшие 3 байта)
    char room_id[7] = {0}; // 6 hex chars + NUL
    if (!room_id_format(room_id, sizeof(room_id))) {
        // Если MAC получить не удалось, отправим 000000, чтобы сервер мог распознать ситуацию
        (void)snprintf(room_id, sizeof(room_id), "000000");
        ESP_LOGW(TAG, "Failed to get base MAC, using room=000000");
    }

    char json_body[64];
    int json_len = snprintf(json_body, sizeof(json_body), "{\"room\":\"%s\"}", room_id);
    if (json_len < 0 || json_len >= (int)sizeof(json_body)) {
        ESP_LOGE(TAG, "Failed to build JSON body for trigger");
        return ESP_FAIL;
    }

    esp_http_client_config_t config_post = {
        .url = "https://camera.boxhoster.com/recorder/trigger",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = 0,
    };
    esp_http_client_handle_t client_post = esp_http_client_init(&config_post);
    // Request JSON so the server responds with 200 JSON instead of HTML redirect.
    esp_http_client_set_header(client_post, "Accept", "application/json");
    esp_http_client_set_header(client_post, "Content-Type", "application/json");
    esp_http_client_set_post_field(client_post, json_body, json_len);
    esp_err_t post_err = esp_http_client_perform(client_post);
    esp_http_client_cleanup(client_post);
    if (post_err == ESP_OK) {
        ESP_LOGI(TAG, "Trigger Sent. room=%s", room_id);
    } else {
        ESP_LOGE(TAG, "Trigger Failed: %s", esp_err_to_name(post_err));
    }
    return post_err;
}
