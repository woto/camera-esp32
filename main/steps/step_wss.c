#include "steps.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "tls/certs.h"
#include "net/ws_handler.h"
#include "util/room_id.h"

#define WS_URL         "wss://volleycam.com/cable"
#define WS_SUBPROTO    "actioncable-v1-json,actioncable-unsupported"
#define WS_IDENTIFIER_ESCAPED "{\\\"channel\\\":\\\"RecordingChannel\\\"}"
#define WS_SUBSCRIBE_MSG "{\"command\":\"subscribe\",\"identifier\":\"" WS_IDENTIFIER_ESCAPED "\"}"
#define WS_BUFFER_SIZE 2048
#define WS_MESSAGE_BUF_SIZE 4096
#define WS_ORIGIN      "https://volleycam.com"

static const char *TAG = "STEP-WSS";
static esp_websocket_client_handle_t s_ws_client = NULL;
static volatile bool s_ws_connected = false;
static char s_ws_msg_buf[WS_MESSAGE_BUF_SIZE];
static size_t s_ws_msg_len = 0;
static size_t s_ws_msg_expected = 0;
static volatile bool s_feed_reload_requested = false;

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "WebSocket connected");
            s_ws_connected = true;
            // Формируем room_id для подписки на канал ActionCable
            char room_id[7] = {0}; // 6 HEX + NUL
            if (!room_id_format(room_id, sizeof(room_id))) {
                // fallback, чтобы бэкенд мог корректно обработать отсутствие MAC
                memcpy(room_id, "000000", 6);
                room_id[6] = '\0';
                ESP_LOGW(TAG, "Failed to get base MAC for WS subscribe, using room=%s", room_id);
            }

            // Собираем сообщение подписки c параметром room внутри identifier
            // Внутренний identifier — это JSON, который передается строкой, поэтому нужны экранированные кавычки
            char subscribe_msg[192];
            int sub_len = snprintf(subscribe_msg, sizeof(subscribe_msg),
                                   "{\"command\":\"subscribe\",\"identifier\":\"{\\\"channel\\\":\\\"RecordingChannel\\\",\\\"room\\\":\\\"%s\\\"}\"}",
                                   room_id);
            if (sub_len < 0 || sub_len >= (int)sizeof(subscribe_msg)) {
                ESP_LOGE(TAG, "Subscribe message build overflow");
                break;
            }
            ESP_LOGI(TAG, "Sending subscribe: %s", subscribe_msg);
            esp_websocket_client_send_text(data->client, subscribe_msg, sub_len, pdMS_TO_TICKS(1000));
            break;
        }
        case WEBSOCKET_EVENT_DISCONNECTED:
            s_ws_connected = false;
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
                    ws_handle_actioncable_message(s_ws_msg_buf, s_ws_msg_len);
                    s_ws_msg_len = 0;
                    s_ws_msg_expected = 0;
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            s_ws_connected = false;
            ESP_LOGW(TAG, "WebSocket error encountered");
            break;
        default:
            break;
    }
}

void step_wss_start(void) {
    if (s_ws_client) {
        return;
    }
    s_ws_connected = false;
    esp_websocket_client_config_t ws_cfg = {
        .uri = WS_URL,
        .cert_pem = CLOUDFLARE_CA_PEM,
        .cert_len = 0,
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

void step_wss_stop(void) {
    if (!s_ws_client) {
        return;
    }
    esp_websocket_client_stop(s_ws_client);
    esp_websocket_client_destroy(s_ws_client);
    s_ws_client = NULL;
    s_ws_connected = false;
}

bool step_wss_is_connected(void) {
    return s_ws_connected;
}

bool step_wss_has_client(void) {
    return s_ws_client != NULL;
}

bool step_wss_take_feed_reload(void) {
    bool flag = s_feed_reload_requested;
    s_feed_reload_requested = false;
    return flag;
}

void step_wss_request_feed_reload(void) {
    s_feed_reload_requested = true;
}

bool step_wss_run(uint16_t *frame_buf, bool wifi_connected, int debug_delay_ms, int error_pause_ms) {
    (void)frame_buf;
    if (!wifi_connected) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));
    step_wss_start();
    for (int i = 0; i < 50; i++) { // ~10 seconds
        if (s_ws_connected) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    step_wss_stop();
    vTaskDelay(pdMS_TO_TICKS(error_pause_ms));
    return false;
}
