#include "steps.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <time.h>

#define TIME_SYNC_BIT BIT0
static EventGroupHandle_t s_time_event_group;
static volatile bool s_time_synced = false;

static void time_sync_notification_cb(struct timeval *tv) {
    (void)tv;
    s_time_synced = true;
    if (s_time_event_group) {
        xEventGroupSetBits(s_time_event_group, TIME_SYNC_BIT);
    }
    ESP_LOGI("STEP-SNTP", "Time synchronized");
}

bool step_sntp_run(uint16_t *frame_buf, bool wifi_connected, int debug_delay_ms, int error_pause_ms) {
    (void)frame_buf;
    if (!wifi_connected) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(debug_delay_ms));

    s_time_event_group = xEventGroupCreate();
    s_time_synced = false;

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    if ((timeinfo.tm_year + 1900) > 2020) {
        s_time_synced = true;
        if (s_time_event_group) {
            xEventGroupSetBits(s_time_event_group, TIME_SYNC_BIT);
        }
    }

    esp_sntp_stop();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();

    EventBits_t time_bits = xEventGroupWaitBits(
        s_time_event_group,
        TIME_SYNC_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000)
    );

    time(&now);
    localtime_r(&now, &timeinfo);
    bool time_ok = (time_bits & TIME_SYNC_BIT) ||
                   sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED ||
                   ((timeinfo.tm_year + 1900) > 2020);
    if (!time_ok) {
        vTaskDelay(pdMS_TO_TICKS(error_pause_ms));
    }
    return time_ok;
}

bool step_sntp_is_synced(void) {
    return s_time_synced;
}

void step_sntp_clear_synced(void) {
    s_time_synced = false;
}
