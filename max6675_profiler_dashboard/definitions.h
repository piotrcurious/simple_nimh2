#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <vector>
#include <cmath>
#include <algorithm>

#ifndef MOCK_TEST
#include <Arduino.h>
#include <freertos/semphr.h>
#include <ESPAsyncWebServer.h>
#else
#include "test_mock/dummy_esp32.h"
#endif

// --- Pin Definitions for MAX6675 Sensors ---
#define MAX6675_SCK_PIN   18
#define MAX6675_MISO_PIN  19
#define MAX6675_CS1_PIN   5
#define MAX6675_CS2_PIN   17
#define MAX6675_CS3_PIN   16
#define MAX6675_CS4_PIN   4

// --- Profiling Definitions ---
static constexpr uint32_t FRAME_PERIOD_US = 100000;   // 10 Hz
static constexpr uint8_t  CORE_COUNT     = 2;
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
extern portMUX_TYPE g_mux;

extern void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags = 0);

// --- Temperature Telemetry Global Variables ---
extern volatile float g_temps[4];
extern volatile bool g_sensorError[4];

// Mutex for WebData
extern SemaphoreHandle_t webDataMutex;
#define WEB_LOCK() if (webDataMutex) xSemaphoreTakeRecursive(webDataMutex, portMAX_DELAY)
#define WEB_UNLOCK() if (webDataMutex) xSemaphoreGiveRecursive(webDataMutex)

#endif // DEFINITIONS_H
