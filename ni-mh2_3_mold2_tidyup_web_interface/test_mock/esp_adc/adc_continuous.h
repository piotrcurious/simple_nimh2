#ifndef ADC_CONTINUOUS_H
#define ADC_CONTINUOUS_H

#include <stdint.h>

typedef void* adc_continuous_handle_t;

typedef struct {
    int max_store_buf_size;
    int conv_frame_size;
} adc_continuous_config_t;

typedef enum {
    ADC_UNIT_1,
    ADC_UNIT_2,
} adc_unit_t;

typedef enum {
    ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    ADC_DIGI_OUTPUT_FORMAT_TYPE2,
} adc_digi_output_format_t;

typedef struct {
    int sample_freq_hz;
    int conv_mode;
    int format;
} adc_continuous_handle_cfg_t;

#endif
