#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_lcd_panel_ops.h"
#define MAX_IMAGES 32

typedef struct {
    char url[256];
    uint8_t *jpeg;
    int jpeg_len;
    bool valid;
    bool loaded;
} image_slot_t;

bool images_init(void);
void images_clear_cache(void);
int images_fetch_digest(bool wifi_connected);
bool images_fetch_and_show(image_slot_t *slot,
                           uint16_t *decode_buf,
                           size_t decode_buf_size,
                           esp_lcd_panel_handle_t panel,
                           bool wifi_connected);
int images_get_count(void);
image_slot_t *images_slots(void);
