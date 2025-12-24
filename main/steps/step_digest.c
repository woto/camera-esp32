#include "steps.h"
#include "images/images.h"

void step_digest_begin(steps_state_t *state, uint16_t *frame_buf) {
    (void)frame_buf;
    if (!state) return;
    state->digest_pending = true;
    state->step = START_STEP_DIGEST;
}

bool step_digest_attempt(steps_state_t *state, uint16_t *frame_buf, int *images_count) {
    (void)frame_buf;
    if (!state || !images_count) {
        return false;
    }
    if (!state->wifi_connected) {
        state->digest_pending = true;
        return false;
    }
    int count = images_fetch_digest(state->wifi_connected);
    if (count > 0) {
        *images_count = count;
        state->digest_pending = false;
        state->step = START_STEP_READY;
        return true;
    }
    state->digest_pending = true;
    return false;
}
