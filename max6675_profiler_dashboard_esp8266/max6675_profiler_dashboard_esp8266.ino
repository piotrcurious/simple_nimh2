#include "definitions_esp8266.h"
#ifndef MOCK_TEST
#include <ESP8266WiFi.h>
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

// --- Web Server ---
#ifndef MOCK_TEST
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
#endif

// --- Profiling Globals ---
CoreBuf g_coreBuf[CORE_COUNT];
volatile uint32_t g_frameSeq = 0;
volatile uint32_t g_frameStartUs = 0;

// --- Record Event ---
void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags) {
    if (core >= CORE_COUNT) return;
    uint8_t idx = g_coreBuf[core].count;
    if (idx < MAX_EVENTS_PER_CORE) {
        g_coreBuf[core].events[idx].taskId = taskId;
        g_coreBuf[core].events[idx].flags  = flags;
        g_coreBuf[core].events[idx].startUs = startUs;
        g_coreBuf[core].events[idx].durUs   = durUs;
        g_coreBuf[core].count = idx + 1;
    }
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

// --- Nonblocking MAX6675 Scheduling ---
void runMAX6675TelemetryTask() {
    static unsigned long lastRun = 0;
    if (millis() - lastRun < 250) return;
    lastRun = millis();

    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(micros() - frameRef);

    const int csPins[4] = { MAX6675_CS1_PIN, MAX6675_CS2_PIN, MAX6675_CS3_PIN, MAX6675_CS4_PIN };
    float temps[4];
    bool errs[4];

    for (int i = 0; i < 4; i++) {
        errs[i] = false;
        temps[i] = readMAX6675(csPins[i], errs[i]);

#ifdef MOCK_TEST
        static float mockPhase = 0.0f;
        temps[i] = 400.0f + 150.0f * sin(mockPhase + i * 0.4f) + 50.0f * cos(mockPhase * 0.3f) + (rand() % 150) / 10.0f;
        errs[i] = false;
        mockPhase += 0.005f; // Oscillate mock values over time
#endif
    }

    for (int i = 0; i < 4; i++) {
        g_temps[i] = temps[i];
        g_sensorError[i] = errs[i];
    }

    uint32_t t1 = (uint32_t)(micros() - frameRef);
    // Record event on Virtual Core 0, Task ID 1 (Sensor), Start, Duration
    recordEvent(0, 1, (uint16_t)t0, (uint16_t)(t1 - t0));
}

// --- Nonblocking Web Telemetry Broadcaster Scheduling ---
void runWebTelemetryBroadcastTask() {
    static unsigned long lastRun = 0;
    if (millis() - lastRun < 1000) return;
    lastRun = millis();

    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(micros() - frameRef);

#ifndef MOCK_TEST
    ws.cleanupClients();
    broadcastLiveTelemetry();
#endif

    uint32_t t1 = (uint32_t)(micros() - frameRef);
    // Record event on Virtual Core 1, Task ID 2 (WebServer), Start, Duration
    recordEvent(1, 2, (uint16_t)t0, (uint16_t)(t1 - t0));
}

// --- Nonblocking Master Profiler Frame Timer Scheduling ---
void runProfilerFrameTask() {
    static unsigned long lastFrameTime = 0;
    if (micros() - lastFrameTime < FRAME_PERIOD_US) return;
    lastFrameTime = micros();

    // 1. Send the compiled virtual lane profiling packet to all connected clients
    sendFramePacket(false);

    // 2. Clear current frame buffers for the new frame
    g_coreBuf[0].count = 0;
    g_coreBuf[1].count = 0;

    g_frameStartUs = micros();
    g_frameSeq++;
}

// --- Setup ---
void setup() {
    Serial.begin(115200);

    // Pin configuration
    pinMode(MAX6675_SCK_PIN, OUTPUT);
    pinMode(MAX6675_MISO_PIN, INPUT);
    const int csPins[4] = { MAX6675_CS1_PIN, MAX6675_CS2_PIN, MAX6675_CS3_PIN, MAX6675_CS4_PIN };
    for (int i = 0; i < 4; i++) {
        pinMode(csPins[i], OUTPUT);
        digitalWrite(csPins[i], HIGH);
    }
    digitalWrite(MAX6675_SCK_PIN, LOW);

#ifndef MOCK_TEST
    WiFi.softAP("MAX6675-ESP8266-Profiler", "password123");
    Serial.print("ESP8266 AP started. IP: ");
    Serial.println(WiFi.softAPIP());

    ws.onEvent(handleWebSocketEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("Async Web Server started on port 80.");
#endif

    g_frameStartUs = micros();
    Serial.println("ESP8266 Standalone System initialized.");
}

// --- Main Loop ---
void loop() {
    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(micros() - frameRef);

    // Cooperative multi-task scheduler
    runMAX6675TelemetryTask();
    runWebTelemetryBroadcastTask();
    runProfilerFrameTask();

    // Yield to the background ESP8266 WiFi driver stack
    yield();

    uint32_t t1 = (uint32_t)(micros() - frameRef);
    // Record event on Virtual Core 1, Task ID 0 (Main Loop idle/schedule), Start, Duration
    recordEvent(1, 0, (uint16_t)t0, (uint16_t)(t1 - t0));
}
