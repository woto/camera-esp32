#include "icons.h"
#include "render/render.h"

static void draw_wifi_icon(uint16_t *frame_buf, int x, int y, bool connected) {
    if (!frame_buf) return;
    const int size = 14;
    uint16_t fg = connected ? 0x07E0 : 0xF800;
    uint16_t bg = 0x0000;
    render_fill_rect(frame_buf, x, y, size, size, bg);
    render_draw_rect_outline(frame_buf, x, y, size, size, 0xFFFF);
    int base_y = y + size - 4;
    render_fill_rect(frame_buf, x + 3, base_y - 2, 2, 4, fg);
    render_fill_rect(frame_buf, x + 6, base_y - 4, 2, 6, fg);
    render_fill_rect(frame_buf, x + 9, base_y - 6, 2, 8, fg);
}

static void draw_ws_icon(uint16_t *frame_buf, int x, int y, bool connected) {
    if (!frame_buf) return;
    const int size = 14;
    uint16_t fg = connected ? 0x07E0 : 0xF800;
    uint16_t bg = 0x0000;
    render_fill_rect(frame_buf, x, y, size, size, bg);
    render_draw_rect_outline(frame_buf, x, y, size, size, 0xFFFF);
    render_draw_rect_outline(frame_buf, x + 3, y + 3, 5, 5, fg);
    render_draw_rect_outline(frame_buf, x + 6, y + 6, 5, 5, fg);
    render_fill_rect(frame_buf, x + 5, y + 5, 2, 2, fg);
}

void icons_draw_status_badges(uint16_t *frame_buf, bool wifi_connected, bool ws_connected) {
    if (!frame_buf) return;
    const int size = 14;
    const int gap = 4;
    int y = 4;
    int x_ws = LCD_H_RES - size - 4;
    int x_wifi = x_ws - gap - size;
    if (x_wifi < 0) x_wifi = 0;
    draw_wifi_icon(frame_buf, x_wifi, y, wifi_connected);
    draw_ws_icon(frame_buf, x_ws, y, ws_connected);
}
