#include "adc_dma.h"
#include "definitions.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>

// ADC configuration constants
// Match reference code exactly
static constexpr uint32_t ADC_SAMPLE_RATE_HZ = 24000;
static constexpr uint32_t CONV_FRAME_SIZE    = 256;
static constexpr uint32_t DMA_POOL_BYTES     = 32768; // 32KB

static constexpr uint32_t FRAME_BUFFER_SIZE   = 128; // Increased
static constexpr uint32_t FRAME_BUFFER_MASK   = FRAME_BUFFER_SIZE - 1;

static constexpr uint32_t SAMPLE_RING_SIZE    = 8192;
static constexpr uint32_t SAMPLE_RING_MASK    = SAMPLE_RING_SIZE - 1;

// Structures
struct TimestampedFrame {
    uint16_t data_size;
    uint8_t  raw_data[CONV_FRAME_SIZE];
};

struct SampleEntry {
    uint16_t raw;
    uint8_t  idx; // Index in AdcChannelIndex
};

struct ChannelAccum {
    uint64_t sum_batch = 0;
    uint32_t count_batch = 0;

    uint64_t sum_total = 0;
    uint32_t count_total = 0;

    float latest_avg_mv = 0.0f;
    uint32_t latest_raw_avg = 0;
};

// Global/static variables
static adc_continuous_handle_t adc_handle = nullptr;
static volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
static volatile uint32_t frame_write_idx = 0;
static volatile uint32_t frame_read_idx  = 0;

static SampleEntry sample_ring[SAMPLE_RING_SIZE];
static uint32_t sample_write_idx = 0;
static uint32_t sample_read_idx  = 0;

static ChannelAccum channel_data[ADC_CH_COUNT];
static int8_t CH_TO_IDX[10];

// Match reference code's exact channels and order
static constexpr uint8_t SCAN_CH_COUNT = 6;
static constexpr adc1_channel_t SCAN_CH[SCAN_CH_COUNT] = {
    ADC1_CHANNEL_0, // GPIO36 -> THERM1
    ADC1_CHANNEL_3, // GPIO39 -> VOLTAGE
    ADC1_CHANNEL_6, // GPIO34 -> CURRENT
    ADC1_CHANNEL_7, // GPIO35 -> THERM_VCC
    ADC1_CHANNEL_4, // GPIO32 -> DUMMY1
    ADC1_CHANNEL_5  // GPIO33 -> DUMMY2
};

static adc_digi_pattern_config_t patterns[SCAN_CH_COUNT];

static esp_adc_cal_characteristics_t adc_chars[ADC_CH_COUNT];
static bool cal_initialized[ADC_CH_COUNT] = {false};

static SemaphoreHandle_t data_mutex = nullptr;

// ISR Callback
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                             const adc_continuous_evt_data_t *edata,
                                             void *user_data) {
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_RELAXED);
    uint32_t rd = __atomic_load_n(&frame_read_idx,  __ATOMIC_ACQUIRE);
    uint32_t next_wr = (wr + 1) & FRAME_BUFFER_MASK;

    if (next_wr == rd) {
        return false;
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;

    memcpy((void*)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);
    frame_buffer[wr].data_size = (uint16_t)sz;

    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE);
    return false;
}

void setupAdcDma() {
    if (data_mutex == nullptr) {
        data_mutex = xSemaphoreCreateMutex();
    }

    for (int i = 0; i < 10; i++) CH_TO_IDX[i] = -1;

    // Only map the channels we care about
    CH_TO_IDX[ADC1_CHANNEL_0] = ADC_IDX_THERM1;
    CH_TO_IDX[ADC1_CHANNEL_3] = ADC_IDX_VOLTAGE;
    CH_TO_IDX[ADC1_CHANNEL_6] = ADC_IDX_CURRENT;
    CH_TO_IDX[ADC1_CHANNEL_7] = ADC_IDX_THERM_VCC;

    // Initialize Calibration (only for real channels)
    // Use DB_12 if available, else DB_11 (DB_12 is 11dB on classic ESP32)
#if defined(ADC_ATTEN_DB_12)
    const adc_atten_t atten = ADC_ATTEN_DB_12;
#else
    const adc_atten_t atten = ADC_ATTEN_DB_11;
#endif

    for (int i = 0; i < ADC_CH_COUNT; i++) {
        esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, 3300, &adc_chars[i]);
        cal_initialized[i] = true;
    }

    // Configure Continuous ADC
    adc_continuous_handle_cfg_t cfg = {};
    cfg.max_store_buf_size = DMA_POOL_BYTES;
    cfg.conv_frame_size = CONV_FRAME_SIZE;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&cfg, &adc_handle));

    // Initialize patterns to exactly match reference code
    memset(patterns, 0, sizeof(patterns));
    for (int i = 0; i < SCAN_CH_COUNT; i++) {
        patterns[i].atten     = atten;
        patterns[i].channel   = SCAN_CH[i];
        patterns[i].unit      = ADC_UNIT_1;
        patterns[i].bit_width = ADC_BITWIDTH_12;
    }

    adc_continuous_config_t dig = {};
    dig.pattern_num = SCAN_CH_COUNT;
    dig.adc_pattern = patterns;
    dig.sample_freq_hz = ADC_SAMPLE_RATE_HZ;
    dig.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig));

    adc_continuous_evt_cbs_t cbs = {};
    cbs.on_conv_done = adc_conv_done_callback;
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, nullptr));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

static inline void pushSample(uint8_t idx, uint16_t raw) {
    sample_ring[sample_write_idx] = { raw, idx };
    uint32_t next = (sample_write_idx + 1) & SAMPLE_RING_MASK;
    if (next == sample_read_idx) {
        sample_read_idx = (sample_read_idx + 1) & SAMPLE_RING_MASK;
    }
    sample_write_idx = next;
}

void processAdcDma() {
    uint32_t rd = __atomic_load_n(&frame_read_idx,  __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);

    while (rd != wr) {
        uint16_t data_size = frame_buffer[rd].data_size;
        int n = data_size / sizeof(adc_digi_output_data_t);
        const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)frame_buffer[rd].raw_data;

        for (int i = 0; i < n; i++) {
            uint8_t ch = p[i].type1.channel;
            if (ch < 10) {
                int8_t idx = CH_TO_IDX[ch];
                if (idx >= 0) {
                    pushSample((uint8_t)idx, (uint16_t)p[i].type1.data);
                }
            }
        }
        rd = (rd + 1) & FRAME_BUFFER_MASK;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        while (sample_read_idx != sample_write_idx) {
            const SampleEntry &e = sample_ring[sample_read_idx];

            channel_data[e.idx].sum_batch += e.raw;
            channel_data[e.idx].count_batch++;

            channel_data[e.idx].sum_total += e.raw;
            channel_data[e.idx].count_total++;

            sample_read_idx = (sample_read_idx + 1) & SAMPLE_RING_MASK;
        }

        for (int i = 0; i < ADC_CH_COUNT; i++) {
            if (channel_data[i].count_batch >= 32) {
                uint32_t avg_raw = (uint32_t)((channel_data[i].sum_batch + (channel_data[i].count_batch / 2)) / channel_data[i].count_batch);
                channel_data[i].latest_raw_avg = avg_raw;
                channel_data[i].latest_avg_mv = (float)esp_adc_cal_raw_to_voltage(avg_raw, &adc_chars[i]);

                channel_data[i].sum_batch = 0;
                channel_data[i].count_batch = 0;
            }
        }
        xSemaphoreGive(data_mutex);
    }
}

float getAdcMillivolts(AdcChannelIndex idx) {
    float val = 0;
    if (data_mutex && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        val = channel_data[idx].latest_avg_mv;
        xSemaphoreGive(data_mutex);
    }
    return val;
}

uint32_t getAdcRawAverage(AdcChannelIndex idx) {
    uint32_t val = 0;
    if (data_mutex && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        val = channel_data[idx].latest_raw_avg;
        xSemaphoreGive(data_mutex);
    }
    return val;
}

void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot) {
    if (data_mutex && xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snapshot.sum = channel_data[idx].sum_total;
        snapshot.count = channel_data[idx].count_total;
        xSemaphoreGive(data_mutex);
    }
}

uint32_t calculateSnapshotAverage(const AdcSnapshot &old_s, const AdcSnapshot &new_s) {
    uint64_t sum_diff = new_s.sum - old_s.sum;
    uint32_t count_diff = new_s.count - old_s.count;
    if (count_diff == 0) return 0;
    return (uint32_t)((sum_diff + (count_diff / 2)) / count_diff);
}

float snapshotToMillivolts(AdcChannelIndex idx, uint32_t avg_raw) {
    if (!cal_initialized[idx]) return 0.0f;
    return (float)esp_adc_cal_raw_to_voltage(avg_raw, &adc_chars[idx]);
}
