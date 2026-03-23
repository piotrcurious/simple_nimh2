#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

static const char* ssid     = "WIFI_SSID";
static const char* password = "WIFI_PASSWORD";

static IPAddress udpAddress(239, 1, 2, 3);
static const uint16_t udpPort = 12345;

static constexpr uint8_t ADC_CH_COUNT = 6;

// Requested channel order:
// 0, 3, 6, 7, 4, 5
// GPIO36, GPIO39, GPIO34, GPIO35, GPIO32, GPIO33
static constexpr adc1_channel_t ADC_CH[ADC_CH_COUNT] = {
    ADC1_CHANNEL_0,
    ADC1_CHANNEL_3,
    ADC1_CHANNEL_6,
    ADC1_CHANNEL_7,
    ADC1_CHANNEL_4,
    ADC1_CHANNEL_5
};

// Map ADC1 channel number -> slot in packet
static constexpr int8_t CH_TO_SLOT[8] = {
    0,  // ch0
   -1,  // ch1
   -1,  // ch2
    1,  // ch3
    4,  // ch4
    5,  // ch5
    2,  // ch6
    3   // ch7
};

static constexpr uint32_t PACKET_HZ = 50;
static constexpr uint64_t PACKET_PERIOD_US = 1000000ULL / PACKET_HZ;

static constexpr uint32_t ADC_SAMPLE_RATE_HZ = 24000;
static constexpr uint32_t CONV_FRAME_SIZE    = 256;
static constexpr uint32_t DMA_POOL_BYTES     = 8192;

static constexpr uint32_t FRAME_BUFFER_SIZE   = 128;
static constexpr uint32_t FRAME_BUFFER_MASK   = FRAME_BUFFER_SIZE - 1;

static constexpr uint32_t SAMPLE_RING_SIZE    = 4096;
static constexpr uint32_t SAMPLE_RING_MASK    = SAMPLE_RING_SIZE - 1;

WiFiUDP udp;
adc_continuous_handle_t adc_handle = nullptr;

struct __attribute__((packed)) UdpPacket {
    uint32_t index;
    uint16_t ch[ADC_CH_COUNT];
};

struct TimestampedFrame {
    uint64_t end_ts_cycles;
    uint16_t data_size;
    uint8_t  raw_data[CONV_FRAME_SIZE];
};

struct SampleEntry {
    uint64_t ts_cycles;
    uint16_t raw;
    uint8_t  slot;
};

struct ChannelAccum {
    uint64_t sum = 0;
    uint32_t count = 0;
};

volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx = 0;
volatile uint32_t frame_read_idx  = 0;
volatile uint32_t frames_dropped  = 0;
volatile uint32_t isr_callback_count = 0;

volatile uint32_t cycle_hi = 0;
volatile uint32_t last_cycle_lo = 0;

static SampleEntry sample_ring[SAMPLE_RING_SIZE];
static uint32_t sample_write_idx = 0;
static uint32_t sample_read_idx  = 0;
static uint32_t samples_dropped   = 0;

static uint16_t latest_raw[ADC_CH_COUNT] = {0};
static ChannelAccum packet_acc[ADC_CH_COUNT];

static uint32_t packet_index = 0;
static uint64_t cpu_freq_hz = 0;
static uint64_t packet_period_cycles = 0;

struct FrameTimingState {
    uint64_t last_frame_end_ts = 0;
    bool initialized = false;
} frame_timing;

static inline uint32_t IRAM_ATTR get_cycle_count32() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a"(ccount));
    return ccount;
}

static inline uint64_t IRAM_ATTR get_cycle_count64_isr() {
    uint32_t lo = get_cycle_count32();
    if (lo < last_cycle_lo) {
        cycle_hi++;
    }
    last_cycle_lo = lo;
    return (uint64_t(cycle_hi) << 32) | lo;
}

static inline uint64_t read_cycle_count64() {
    uint32_t hi1, hi2, lo;
    do {
        hi1 = cycle_hi;
        lo = get_cycle_count32();
        hi2 = cycle_hi;
    } while (hi1 != hi2);
    return (uint64_t(hi1) << 32) | lo;
}

static inline void pushSample(uint8_t slot, uint16_t raw, uint64_t ts_cycles) {
    sample_ring[sample_write_idx] = { ts_cycles, raw, slot };
    latest_raw[slot] = raw;

    uint32_t next = (sample_write_idx + 1) & SAMPLE_RING_MASK;
    if (next == sample_read_idx) {
        sample_read_idx = (sample_read_idx + 1) & SAMPLE_RING_MASK;
        samples_dropped++;
    }
    sample_write_idx = next;
}

static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                             const adc_continuous_evt_data_t *edata,
                                             void *user_data) {
    (void)handle;
    (void)user_data;

    uint64_t ts = get_cycle_count64_isr();

    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_RELAXED);
    uint32_t rd = __atomic_load_n(&frame_read_idx,  __ATOMIC_ACQUIRE);
    uint32_t next_wr = (wr + 1) & FRAME_BUFFER_MASK;

    if (next_wr == rd) {
        rd = (rd + 1) & FRAME_BUFFER_MASK;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
        __atomic_fetch_add(&frames_dropped, 1u, __ATOMIC_RELAXED);
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;

    memcpy((void*)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);
    frame_buffer[wr].end_ts_cycles = ts;
    frame_buffer[wr].data_size = (uint16_t)sz;

    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE);
    __atomic_fetch_add(&isr_callback_count, 1u, __ATOMIC_RELAXED);
    return false;
}

static void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
    }

    udp.begin(0);
}

static void setupADC() {
    adc_continuous_handle_cfg_t cfg = {};
    cfg.max_store_buf_size = DMA_POOL_BYTES;
    cfg.conv_frame_size = CONV_FRAME_SIZE;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&cfg, &adc_handle));

    static adc_digi_pattern_config_t patterns[ADC_CH_COUNT];
    for (int i = 0; i < ADC_CH_COUNT; ++i) {
        patterns[i].atten     = ADC_ATTEN_DB_12;
        patterns[i].channel   = ADC_CH[i];
        patterns[i].unit      = ADC_UNIT_1;
        patterns[i].bit_width = ADC_BITWIDTH_12;
    }

    adc_continuous_config_t dig = {};
    dig.pattern_num    = ADC_CH_COUNT;
    dig.adc_pattern    = patterns;
    dig.sample_freq_hz = ADC_SAMPLE_RATE_HZ;
    dig.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
    dig.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig));

    adc_continuous_evt_cbs_t cbs = {};
    cbs.on_conv_done = adc_conv_done_callback;
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, nullptr));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

static inline void processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx,  __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);
    if (rd == wr) return;

    while (rd != wr) {
        const TimestampedFrame &f = frame_buffer[rd];
        const int n = (int)(f.data_size / sizeof(adc_digi_output_data_t));

        if (n > 0) {
            const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)f.raw_data;

            if (!frame_timing.initialized) {
                frame_timing.last_frame_end_ts = f.end_ts_cycles;
                frame_timing.initialized = true;
            } else {
                uint64_t frame_start_ts = frame_timing.last_frame_end_ts;
                uint64_t elapsed_cycles = f.end_ts_cycles - frame_timing.last_frame_end_ts;
                if (elapsed_cycles == 0) elapsed_cycles = 1;

                for (int i = 0; i < n; ++i) {
                    if (p[i].type1.unit != ADC_UNIT_1) continue;

                    uint8_t ch = p[i].type1.channel;
                    if (ch >= 8) continue;

                    int8_t slot = CH_TO_SLOT[ch];
                    if (slot < 0) continue;

                    uint64_t sample_ts =
                        frame_start_ts + (uint64_t(i + 1) * elapsed_cycles) / (uint64_t)n;

                    pushSample((uint8_t)slot, (uint16_t)p[i].type1.data, sample_ts);
                }

                frame_timing.last_frame_end_ts = f.end_ts_cycles;
            }
        }

        rd = (rd + 1) & FRAME_BUFFER_MASK;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

static inline void drainSamplesToCutoff(uint64_t cutoff_cycles) {
    while (sample_read_idx != sample_write_idx) {
        const SampleEntry &e = sample_ring[sample_read_idx];
        if (e.ts_cycles > cutoff_cycles) break;

        packet_acc[e.slot].sum += (uint64_t)e.raw;
        packet_acc[e.slot].count++;

        sample_read_idx = (sample_read_idx + 1) & SAMPLE_RING_MASK;
    }
}

static inline void sendOversampledPacket() {
    UdpPacket pkt;
    pkt.index = packet_index++;

    for (int i = 0; i < ADC_CH_COUNT; ++i) {
        const uint64_t cnt = packet_acc[i].count;
        if (cnt != 0) {
            // Rounded integer mean, safe from overflow for this workload.
            uint64_t avg = (packet_acc[i].sum + (cnt >> 1)) / cnt;
            if (avg > 65535ULL) avg = 65535ULL;
            pkt.ch[i] = (uint16_t)avg;
        } else {
            pkt.ch[i] = latest_raw[i];
        }

        packet_acc[i].sum = 0;
        packet_acc[i].count = 0;
    }

    if (WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(udpAddress, udpPort);
        udp.write((const uint8_t*)&pkt, sizeof(pkt));
        udp.endPacket();
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    setupWiFi();

    cpu_freq_hz = (uint64_t)ESP.getCpuFreqMHz() * 1000000ULL;
    packet_period_cycles = cpu_freq_hz / PACKET_HZ;

    setupADC();

    Serial.printf("CPU=%llu Hz, packet period=%llu cycles\n",
                  (unsigned long long)cpu_freq_hz,
                  (unsigned long long)packet_period_cycles);
}

void loop() {
    processIncomingDMA();

    static uint64_t next_send_cycles = 0;
    uint64_t now_cycles = read_cycle_count64();

    if (next_send_cycles == 0) {
        next_send_cycles = now_cycles + packet_period_cycles;
    }

    if ((int64_t)(now_cycles - next_send_cycles) >= 0) {
        drainSamplesToCutoff(now_cycles);
        sendOversampledPacket();

        next_send_cycles += packet_period_cycles;
        if ((int64_t)(now_cycles - next_send_cycles) >= 0) {
            next_send_cycles = now_cycles + packet_period_cycles;
        }
    }

    yield();
}
