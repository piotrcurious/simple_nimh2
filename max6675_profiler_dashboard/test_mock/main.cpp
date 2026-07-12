#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>

#include "dummy_esp32.h"

// Forward definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define WEBSERVER_H

#include "../definitions.h"

MockSerial Serial;
unsigned long mock_millis = 0;

// Global symbols from .ino/cpp
extern void broadcastLiveTelemetry();
extern void sendFramePacket(bool timeoutFlag);
extern void setup();
extern void loop();

#ifndef MOCK_TEST
AsyncWebServer server(80);
#endif
AsyncWebSocket ws("/ws");
SemaphoreHandle_t webDataMutex = NULL;

void AsyncWebServerRequest::send(int code, const char* type, String content) {}
void AsyncWebServerRequest::send(AsyncWebServerResponse* response) { delete response; }

void test_live_telemetry() {
    std::cout << "Testing live telemetry serialization..." << std::endl;

    // Set some dummy temperature values
    g_temps[0] = 234.50f;
    g_temps[1] = 456.75f;
    g_temps[2] = 567.00f;
    g_temps[3] = 789.25f;
    g_sensorError[0] = false;
    g_sensorError[1] = false;
    g_sensorError[2] = false;
    g_sensorError[3] = false;

    broadcastLiveTelemetry();

    std::cout << "  Live JSON Telemetry: " << ws.lastTextAll << std::endl;
    assert(ws.lastTextAll.find("234.50") != std::string::npos);
    assert(ws.lastTextAll.find("456.75") != std::string::npos);
    assert(ws.lastTextAll.find("e1\":0") != std::string::npos);
    std::cout << "test_live_telemetry PASSED" << std::endl << std::endl;
}

void test_profiling_timeline_stream() {
    std::cout << "Testing binary profiling packet stream..." << std::endl;

    // Simulate record events on Core 0 and Core 1
    recordEvent(0, 1, 120, 450); // Task ID 1, Start 120, Duration 450
    recordEvent(1, 0, 50, 100);  // Task ID 0, Start 50, Duration 100
    recordEvent(1, 2, 600, 250); // Task ID 2, Start 600, Duration 250

    sendFramePacket(false);

    // Verify binary packet
    assert(ws._clients[0].lastBinary.size() >= 18 + 3 * 6);
    uint16_t magic = ws._clients[0].lastBinary[0] | (ws._clients[0].lastBinary[1] << 8);
    assert(magic == 0x5450); // "TP"

    uint8_t c0 = ws._clients[0].lastBinary[14];
    uint8_t c1 = ws._clients[0].lastBinary[15];
    assert(c0 == 1);
    assert(c1 == 2);

    std::cout << "  Magic number: 0x" << std::hex << magic << std::dec << " (TP)" << std::endl;
    std::cout << "  Core 0 Events: " << (int)c0 << std::endl;
    std::cout << "  Core 1 Events: " << (int)c1 << std::endl;
    std::cout << "test_profiling_timeline_stream PASSED" << std::endl << std::endl;
}

int main() {
    test_live_telemetry();
    test_profiling_timeline_stream();
    std::cout << "All local MAX6675 simulation tests PASSED successfully!" << std::endl;
    return 0;
}
