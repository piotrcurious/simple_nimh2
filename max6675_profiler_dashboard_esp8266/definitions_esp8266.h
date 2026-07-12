#ifndef DEFINITIONS_ESP8266_H
#define DEFINITIONS_ESP8266_H

#include <vector>
#include <cmath>
#include <algorithm>

#ifndef MOCK_TEST
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#else
#include "test_mock/dummy_esp8266_server.h"
#endif

// --- Pin Definitions for ESP8266 MAX6675 ---
// NodeMCU / D1 Mini pins:
#define MAX6675_SCK_PIN   14 // D5 (SCK)
#define MAX6675_MISO_PIN  12 // D6 (MISO)
#define MAX6675_CS1_PIN   5  // D1
#define MAX6675_CS2_PIN   4  // D2
#define MAX6675_CS3_PIN   0  // D3
#define MAX6675_CS4_PIN   2  // D4

// --- Profiling Definitions ---
static constexpr uint32_t FRAME_PERIOD_US = 100000;   // 10 Hz
static constexpr uint8_t  CORE_COUNT     = 2;         // 2 Virtual lanes for single core
static constexpr uint8_t  MAX_EVENTS_PER_CORE = 16;

struct __attribute__((packed)) EventRec {
    uint8_t  taskId;
    uint8_t  flags;
    uint16_t startUs;
    uint16_t durUs;
};

struct CoreBuf {
    volatile uint8_t count;
    EventRec events[MAX_EVENTS_PER_CORE];
};

extern CoreBuf g_coreBuf[CORE_COUNT];
extern volatile uint32_t g_frameStartUs;
extern volatile uint32_t g_frameSeq;

extern void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags = 0);

// --- Temperature Telemetry Global Variables ---
extern volatile float g_temps[4];
extern volatile bool g_sensorError[4];

#endif // DEFINITIONS_ESP8266_H
