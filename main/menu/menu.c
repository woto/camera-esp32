#include "menu.h"
#include "icons/icons.h"
#include "render/render.h"
#include <stdio.h>
#include "esp_log.h"
#include "util/room_id.h"

void menu_render(esp_lcd_panel_handle_t panel, uint16_t *frame_buf, int selected, int bat_mv, bool wifi_connected) {
    (void)panel;
    if (!frame_buf) return;
    render_framebuf_fill_color(frame_buf, 0x0000);
    const int box_w = LCD_H_RES - 30;
    const int box_h = 40;
    const int gap = 10;
    const int total_h = MENU_ITEMS * box_h + (MENU_ITEMS - 1) * gap;
    const int start_y = (LCD_V_RES - total_h) / 2;
    const int start_x = 15;
    char voltage_label[24];
    char device_id_label[24];
    if (bat_mv > 0) {
        snprintf(voltage_label, sizeof(voltage_label), "VOLT %d.%02dV", bat_mv / 1000, (bat_mv % 1000) / 10);
    } else {
        snprintf(voltage_label, sizeof(voltage_label), "VOLT --.-V");
    }

    // Формируем короткий уникальный идентификатор по MAC (последние 3 байта)
    char short_id[7] = {0};
    if (room_id_format(short_id, sizeof(short_id))) {
        snprintf(device_id_label, sizeof(device_id_label), "ID %s", short_id);
    } else {
        snprintf(device_id_label, sizeof(device_id_label), "ID 000000");
    }
    const char *labels[MENU_ITEMS] = {
        voltage_label,    // 0: Battery voltage (not selectable)
        device_id_label,  // 1: MAC/Device ID (not selectable)
        "FAST SLEEP",     // 2: Selectable
        "DEEP SLEEP",     // 3: Selectable
    };
    for (int i = 0; i < MENU_ITEMS; i++) {
        int y = start_y + i * (box_h + gap);
        if (i == selected) {
            render_fill_rect(frame_buf, start_x, y, box_w, box_h, 0xFFFF);
            render_draw_rect_outline(frame_buf, start_x, y, box_w, box_h, 0x0000);
            render_draw_text(frame_buf, start_x + 10, y + 12, labels[i], 0x0000);
        } else {
            render_draw_rect_outline(frame_buf, start_x, y, box_w, box_h, 0xFFFF);
            render_draw_text(frame_buf, start_x + 10, y + 12, labels[i], 0xFFFF);
        }
    }
    icons_draw_status_badges(frame_buf, wifi_connected);
    render_panel_draw_bitmap_chunked(panel, frame_buf, LCD_H_RES, LCD_V_RES);
}
