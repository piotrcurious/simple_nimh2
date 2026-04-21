#ifndef ESP_ADC_CAL_H
#define ESP_ADC_CAL_H

#include <stdint.h>

typedef enum {
    ADC_ATTEN_DB_0,
    ADC_ATTEN_DB_2_5,
    ADC_ATTEN_DB_6,
    ADC_ATTEN_DB_11,
} adc_atten_t;

typedef struct {
    uint32_t vref;
} esp_adc_cal_characteristics_t;

#endif
