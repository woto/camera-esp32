#include "room_id.h"
#include "esp_mac.h"
#include <stdio.h>

bool room_id_format(char *out, size_t out_len) {
    if (!out || out_len < 7) {
        return false; // требуется 6 символов + NUL
    }
    uint8_t mac[6] = {0};
    if (esp_base_mac_addr_get(mac) != ESP_OK) {
        return false;
    }
    int n = snprintf(out, out_len, "%02X%02X%02X", mac[3], mac[4], mac[5]);
    return n == 6;
}
