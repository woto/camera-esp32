#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_event.h"

typedef enum {
    START_STEP_WIFI = 0,
    START_STEP_SNTP,
    START_STEP_DIGEST,
    START_STEP_READY
} startup_step_t;

typedef struct {
    startup_step_t step;
    bool wifi_connected;
    bool time_synced;
    bool digest_pending;
} steps_state_t;

void step_wifi_begin(steps_state_t *state, uint16_t *frame_buf);
void step_wifi_retry(steps_state_t *state, uint16_t *frame_buf);

bool step_sntp_run(uint16_t *frame_buf, bool wifi_connected, int debug_delay_ms, int error_pause_ms);
bool step_sntp_is_synced(void);
void step_sntp_clear_synced(void);

void step_digest_begin(steps_state_t *state, uint16_t *frame_buf);
bool step_digest_attempt(steps_state_t *state, uint16_t *frame_buf, int *images_count);
