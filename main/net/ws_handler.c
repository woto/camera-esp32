#include "ws_handler.h"
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_log.h"
#include "steps/steps.h"

static const char *TAG = "WS-HANDLER";

void ws_handle_actioncable_message(const char *payload, size_t len) {
    if (!payload || len == 0) {
        return;
    }
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
            step_wss_request_feed_reload();
        }
    }
    cJSON_Delete(root);
}
