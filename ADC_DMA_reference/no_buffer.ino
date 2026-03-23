#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_adc/adc_continuous.h"

static const char *SSID     = "WIFI_SSID";
static const char *PASSWORD = "WIFI_PASSWORD";

// Multicast or unicast target
static IPAddress UDP_ADDR(239, 1, 2, 3);
static constexpr uint16_t UDP_PORT = 12345;

static constexpr uint8_t  ADC_CH_COUNT = 6;
static constexpr uint32_t  PACKET_HZ = 50;
static constexpr uint32_t  PACKET_PERIOD_MS = 1000 / PACKET_HZ;

// Total scan rate across all 6 channels.
// For a 50 Hz packet rate, this is plenty for fresh raw samples.
static constexpr uint32_t ADC_SAMPLE_RATE_HZ = 6000;

// DMA buffer sizing
static constexpr uint32_t ADC_FRAME_SIZE_BYTES = 256;
static constexpr uint32_t ADC_POOL_BYTES       = 2048;
static constexpr uint32_t ADC_READ_BUF_BYTES   = 256;

// Packet = index + 6 raw channel values
struct __attribute__((packed)) AdcUdpPacket {
  uint32_t index;
  uint16_t ch[ADC_CH_COUNT];
};

static_assert(sizeof(AdcUdpPacket) == 16, "Unexpected packet size");

static WiFiUDP udp;
static adc_continuous_handle_t adc_handle = nullptr;

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_latest[ADC_CH_COUNT] = {0};
static uint32_t g_packet_index = 0;

// Packet order requested by you: 0, 3, 6, 7, 4, 5
static constexpr adc1_channel_t kChannels[ADC_CH_COUNT] = {
  ADC1_CHANNEL_0, // GPIO36
  ADC1_CHANNEL_3, // GPIO39
  ADC1_CHANNEL_6, // GPIO34
  ADC1_CHANNEL_7, // GPIO35
  ADC1_CHANNEL_4, // GPIO32
  ADC1_CHANNEL_5  // GPIO33
};

// Map ADC1 channel number (0..7) to packet slot, -1 = unused
static constexpr int8_t kChannelToSlot[8] = {
  0,  // ch0
 -1,  // ch1
 -1,  // ch2
  1,  // ch3
  4,  // ch4
  5,  // ch5
  2,  // ch6
  3   // ch7
};

static void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
  }

  udp.begin(0); // local ephemeral port
}

static void setupADC() {
  adc_continuous_handle_cfg_t handle_cfg = {};
  handle_cfg.max_store_buf_size = ADC_POOL_BYTES;
  handle_cfg.conv_frame_size     = ADC_FRAME_SIZE_BYTES;

  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

  static adc_digi_pattern_config_t patterns[ADC_CH_COUNT];
  for (int i = 0; i < ADC_CH_COUNT; ++i) {
    patterns[i].atten     = ADC_ATTEN_DB_12;
    patterns[i].channel   = kChannels[i];
    patterns[i].unit      = ADC_UNIT_1;
    patterns[i].bit_width = ADC_BITWIDTH_12;
  }

  adc_continuous_config_t cfg = {};
  cfg.pattern_num    = ADC_CH_COUNT;
  cfg.adc_pattern    = patterns;
  cfg.sample_freq_hz = ADC_SAMPLE_RATE_HZ;
  cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  cfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1;

  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cfg));
  ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

static void adcReadTask(void *pv) {
  uint8_t raw[ADC_READ_BUF_BYTES];

  for (;;) {
    uint32_t out_length = 0;
    esp_err_t ret = adc_continuous_read(
      adc_handle,
      raw,
      sizeof(raw),
      &out_length,
      portMAX_DELAY
    );

    if (ret != ESP_OK || out_length == 0) {
      continue;
    }

    const adc_digi_output_data_t *samples =
      reinterpret_cast<const adc_digi_output_data_t *>(raw);

    uint32_t n = out_length / sizeof(adc_digi_output_data_t);
    for (uint32_t i = 0; i < n; ++i) {
      const auto &s = samples[i];

      if (s.type1.unit != ADC_UNIT_1) {
        continue;
      }

      uint8_t ch = s.type1.channel;
      if (ch < 8) {
        int8_t slot = kChannelToSlot[ch];
        if (slot >= 0) {
          portENTER_CRITICAL(&g_mux);
          g_latest[slot] = s.type1.data;
          portEXIT_CRITICAL(&g_mux);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  setupWiFi();
  setupADC();

  // ADC drain task
  xTaskCreatePinnedToCore(
    adcReadTask,
    "adcReadTask",
    4096,
    nullptr,
    2,
    nullptr,
    0
  );
}

void loop() {
  static uint32_t last_send_ms = 0;
  uint32_t now = millis();

  if ((uint32_t)(now - last_send_ms) < PACKET_PERIOD_MS) {
    delay(1);
    return;
  }
  last_send_ms += PACKET_PERIOD_MS;

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  AdcUdpPacket pkt;
  pkt.index = g_packet_index++;

  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < ADC_CH_COUNT; ++i) {
    pkt.ch[i] = g_latest[i];
  }
  portEXIT_CRITICAL(&g_mux);

  udp.beginPacket(UDP_ADDR, UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
  udp.endPacket();
}
