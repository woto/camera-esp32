#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "display_config.h"

void icons_draw_status_badges(uint16_t *frame_buf, bool wifi_connected);
