#pragma once
#include <stdbool.h>

void battery_init(void);
int battery_read_mv(void);
bool battery_cali_enabled(void);
