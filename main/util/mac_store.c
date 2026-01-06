#include "mac_store.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs.h"

#define MAC_STORE_NAMESPACE "net"
#define MAC_STORE_KEY "user_mac"

static const char *TAG = "MAC_STORE";

bool mac_store_load(uint8_t mac[6]) {
    if (!mac) {
        return false;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MAC_STORE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }
    size_t len = 6;
    err = nvs_get_blob(handle, MAC_STORE_KEY, mac, &len);
    nvs_close(handle);
    return err == ESP_OK && len == 6;
}

esp_err_t mac_store_save(const uint8_t mac[6]) {
    if (!mac) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MAC_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, MAC_STORE_KEY, mac, 6);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t mac_store_apply(void) {
    uint8_t mac[6];
    if (!mac_store_load(mac)) {
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t err = esp_base_mac_addr_set(mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to apply stored MAC: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Applied stored MAC");
    }
    return err;
}

void mac_generate_random(uint8_t mac[6]) {
    if (!mac) {
        return;
    }
    esp_fill_random(mac, 6);
    mac[0] = (mac[0] | 0x02) & 0xFE; // locally administered, unicast
}
