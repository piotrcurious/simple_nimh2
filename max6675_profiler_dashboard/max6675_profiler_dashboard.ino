#include "definitions.h"
#ifndef MOCK_TEST
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#endif

// --- External Handlers / Functions ---
extern void handleRoot(AsyncWebServerRequest *request);
extern void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
extern void broadcastLiveTelemetry();
extern void sendFramePacket(bool timeoutFlag);

// --- Telemetry and State ---
volatile float g_temps[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
volatile bool g_sensorError[4] = { true, true, true, true };

// --- Web Server & Mutex ---
#ifndef MOCK_TEST
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
SemaphoreHandle_t webDataMutex = NULL;
#endif

// --- Profiling Globals ---
CoreBuf g_coreBuf[CORE_COUNT];
volatile uint32_t g_frameSeq = 0;
volatile uint32_t g_frameStartUs = 0;
TaskHandle_t g_masterTaskHandle = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// --- Record Event ---
void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags) {
    if (core >= CORE_COUNT) return;
    portENTER_CRITICAL(&g_mux);
    uint8_t idx = g_coreBuf[core].count;
    if (idx < MAX_EVENTS_PER_CORE) {
        g_coreBuf[core].events[idx].taskId = taskId;
        g_coreBuf[core].events[idx].flags  = flags;
        g_coreBuf[core].events[idx].startUs = startUs;
        g_coreBuf[core].events[idx].durUs   = durUs;
        g_coreBuf[core].count = idx + 1;
    }
    portEXIT_CRITICAL(&g_mux);
}

// --- Software SPI Reader for MAX6675 ---
float readMAX6675(int csPin, bool &error) {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, LOW);
    delayMicroseconds(1); // Settle time

    uint16_t value = 0;
    for (int i = 0; i < 16; i++) {
        digitalWrite(MAX6675_SCK_PIN, HIGH);
        delayMicroseconds(1);
        value <<= 1;
        if (digitalRead(MAX6675_MISO_PIN)) {
            value |= 1;
        }
        digitalWrite(MAX6675_SCK_PIN, LOW);
        delayMicroseconds(1);
    }

    digitalWrite(csPin, HIGH);

    // Bit 2: High means open thermocouple input
    if (value & 4) {
        error = true;
        return 0.0f;
    }

    error = false;
    uint16_t tempRaw = (value >> 3) & 0x0FFF;
    return tempRaw * 0.25f;
}

// --- Periodic MAX6675 Reading Task (Core 0) ---
void task_readMAX6675(void* parameter) {
    const int csPins[4] = { MAX6675_CS1_PIN, MAX6675_CS2_PIN, MAX6675_CS3_PIN, MAX6675_CS4_PIN };

    // Configure SPI pin modes
    pinMode(MAX6675_SCK_PIN, OUTPUT);
    pinMode(MAX6675_MISO_PIN, INPUT);
    for (int i = 0; i < 4; i++) {
        pinMode(csPins[i], OUTPUT);
        digitalWrite(csPins[i], HIGH);
    }
    digitalWrite(MAX6675_SCK_PIN, LOW);

#ifdef MOCK_TEST
    static float mockPhase = 0.0f;
#endif

    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);

        float temps[4];
        bool errs[4];

        for (int i = 0; i < 4; i++) {
            errs[i] = false;
            temps[i] = readMAX6675(csPins[i], errs[i]);

#ifdef MOCK_TEST
            // Simulated realistic temperature swings in the range of 200C - 800C
            temps[i] = 400.0f + 150.0f * sin(mockPhase + i * 0.4f) + 50.0f * cos(mockPhase * 0.3f) + (rand() % 150) / 10.0f;
            errs[i] = false;
#endif
        }

#ifdef MOCK_TEST
        mockPhase += 0.02f;
#endif

        WEB_LOCK();
        for (int i = 0; i < 4; i++) {
            g_temps[i] = temps[i];
            g_sensorError[i] = errs[i];
        }
        WEB_UNLOCK();

        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        // Record event: Core 0, Task ID 1 (MAX6675), Start Time, Duration
        recordEvent(0, 1, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(250)); // MAX6675 conversion cycle is 220ms
    }
}

// --- WebSocket Cleanup and Telemetry Broadcasting Task (Core 1) ---
void task_webServer(void* parameter) {
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);

#ifndef MOCK_TEST
        ws.cleanupClients();
        broadcastLiveTelemetry();
#endif

        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        // Record event: Core 1, Task ID 2 (WebServer Telemetry), Start Time, Duration
        recordEvent(1, 2, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(1000)); // Every 1 second
    }
}

// --- Master Profiler Frame Timer Task (Core 1) ---
static void masterTask(void *param) {
    (void)param;
    for (;;) {
        const uint32_t frameStart = (uint32_t)esp_timer_get_time();

        // Reset per-frame buffers
        portENTER_CRITICAL(&g_mux);
        g_coreBuf[0].count = 0;
        g_coreBuf[1].count = 0;
        portEXIT_CRITICAL(&g_mux);

        g_frameStartUs = frameStart;
        g_frameSeq++;

        // Sleep for the frame duration (10Hz = 100ms)
        vTaskDelay(pdMS_TO_TICKS(FRAME_PERIOD_US / 1000));

        // Send profiling timeline package to all active clients
        sendFramePacket(false);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);

#ifndef MOCK_TEST
    webDataMutex = xSemaphoreCreateRecursiveMutex();
#endif

    // Set up AP
#ifndef MOCK_TEST
    WiFi.softAP("MAX6675-Profiler", "password123");
    Serial.print("Access Point started. IP: ");
    Serial.println(WiFi.softAPIP());
#endif

#ifndef MOCK_TEST
    ws.onEvent(handleWebSocketEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("Async Web Server started on port 80.");
#endif

    // Start Nonblocking Tasks
    xTaskCreatePinnedToCore(task_readMAX6675, "ReadMAX6675", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_webServer, "WebServer", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(masterTask, "MasterProfiler", 4096, NULL, 1, &g_masterTaskHandle, 1);

#ifndef MOCK_TEST
    WiFi.setSleep(false); // Disable sleep mode to keep connection snappy
#endif
    Serial.println("System initialized successfully.");
}

// --- Main Application Loop (Core 1) ---
void loop() {
    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);

    // Core 1 main loop idle yielding
    delay(10);

    uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
    // Record event: Core 1, Task ID 0 (Main Loop), Start Time, Duration
    recordEvent(1, 0, (uint16_t)t0, (uint16_t)(t1 - t0));
}
