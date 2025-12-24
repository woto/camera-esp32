#include "battery.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "display_config.h"

#define BAT_ADC_UNIT ADC_UNIT_1
#define BAT_ADC_CHANNEL ADC_CHANNEL_3
#define BAT_ADC_ATTEN ADC_ATTEN_DB_12
#define BAT_VOLT_DIVIDER_NUM 2
#define BAT_VOLT_DIVIDER_DEN 1

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_cali_enabled = false;

void battery_init(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    if (adc_oneshot_new_unit(&init_cfg, &s_adc_handle) != ESP_OK) {
        s_adc_handle = NULL;
        return;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = BAT_ADC_ATTEN
    };
    if (adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg) != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return;
    }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = BAT_ADC_UNIT,
            .atten = BAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
            s_adc_cali_enabled = true;
        }
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = BAT_ADC_UNIT,
            .atten = BAT_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
            s_adc_cali_enabled = true;
        }
    }
#endif
}

int battery_read_mv(void) {
    if (!s_adc_handle) {
        return -1;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return -1;
    }
    if (s_adc_cali_enabled) {
        int voltage_mv = 0;
        if (adc_cali_raw_to_voltage(s_adc_cali, raw, &voltage_mv) == ESP_OK) {
            return (voltage_mv * BAT_VOLT_DIVIDER_NUM) / BAT_VOLT_DIVIDER_DEN;
        }
    }
    return ((raw * 1100) / 4095) * BAT_VOLT_DIVIDER_NUM / BAT_VOLT_DIVIDER_DEN;
}

bool battery_cali_enabled(void) {
    return s_adc_cali_enabled;
}
