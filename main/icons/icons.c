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

void icons_draw_status_badges(uint16_t *frame_buf, bool wifi_connected) {
    if (!frame_buf) return;
    const int size = 14;
    int y = 4;
    int x_wifi = LCD_H_RES - size - 4;
    if (x_wifi < 0) x_wifi = 0;
    draw_wifi_icon(frame_buf, x_wifi, y, wifi_connected);
}
