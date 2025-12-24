#include "steps.h"

void step_wifi_begin(steps_state_t *state, uint16_t *frame_buf) {
    (void)frame_buf;
    if (!state) return;
    state->wifi_connected = false;
    state->step = START_STEP_WIFI;
}

void step_wifi_retry(steps_state_t *state, uint16_t *frame_buf) {
    (void)frame_buf;
    if (!state) return;
    state->wifi_connected = false;
}
