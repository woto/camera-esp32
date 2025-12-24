#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_lcd_panel_ops.h"
#include "display_config.h"

bool render_panel_draw_bitmap_chunked(esp_lcd_panel_handle_t panel, const uint16_t *buf, int buf_width, int buf_height);
void render_fill_color(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, uint16_t color);
void render_framebuf_fill_color(uint16_t *frame_buf, uint16_t color);
void render_fill_rect(uint16_t *frame_buf, int x, int y, int w, int h, uint16_t color);
void render_draw_rect_outline(uint16_t *frame_buf, int x, int y, int w, int h, uint16_t color);
void render_draw_text(uint16_t *frame_buf, int x, int y, const char *text, uint16_t color);
void render_status_screen(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, const char *line1, const char *line2, uint16_t bg_color, uint16_t fg_color, bool wifi_connected, bool ws_connected);
