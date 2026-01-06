#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

bool mac_store_load(uint8_t mac[6]);
esp_err_t mac_store_save(const uint8_t mac[6]);
esp_err_t mac_store_apply(void);
void mac_generate_random(uint8_t mac[6]);
