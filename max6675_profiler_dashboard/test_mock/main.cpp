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
extern float readMAX6675(int csPin, bool &error);

#ifndef MOCK_TEST
AsyncWebServer server(80);
#endif
AsyncWebSocket ws("/ws");
SemaphoreHandle_t webDataMutex = NULL;

int (*mock_digitalRead_cb)(int pin) = nullptr;

void AsyncWebServerRequest::send(int code, const char* type, String content) {}
void AsyncWebServerRequest::send(AsyncWebServerResponse* response) { delete response; }

// --- SPI bits streamer mock helper ---
static uint16_t g_spi_mock_word = 0;
static int g_spi_bit_index = 0;

int mock_spi_digitalRead(int pin) {
    if (pin == MAX6675_MISO_PIN) {
        int bit = (g_spi_mock_word >> (15 - g_spi_bit_index)) & 1;
        g_spi_bit_index = (g_spi_bit_index + 1) % 16;
        return bit;
    }
    return 0;
}

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

void test_max6675_decoding() {
    std::cout << "Testing MAX6675 decoding logic with mock SPI stream..." << std::endl;

    mock_digitalRead_cb = mock_spi_digitalRead;

    // 1. Let's test a successful decoding of 350.5°C
    // raw temperature = 350.5 / 0.25 = 1402
    // MAX6675 format: bit 15: dummy, bit 14-3: raw temperature, bit 2: thermocouple input open, bit 1: device ID, bit 0: state
    // Let's form the word:
    // Raw temp in bits 14-3 = 1402 << 3
    uint16_t raw_temp = 1402;
    g_spi_mock_word = (raw_temp << 3) & 0x7FF8;
    g_spi_bit_index = 0;

    bool error = false;
    float temp = readMAX6675(MAX6675_CS1_PIN, error);

    std::cout << "  Decoded Temp: " << temp << " °C, Error: " << (error ? "TRUE" : "FALSE") << std::endl;
    assert(!error);
    assert(std::abs(temp - 350.50f) < 0.01f);

    std::cout << "test_max6675_decoding PASSED" << std::endl << std::endl;
}

void test_sensor_errors() {
    std::cout << "Testing MAX6675 open thermocouple input error detection..." << std::endl;

    mock_digitalRead_cb = mock_spi_digitalRead;

    // Set bit 2 to 1 (Thermocouple open error)
    g_spi_mock_word = (1 << 2);
    g_spi_bit_index = 0;

    bool error = false;
    float temp = readMAX6675(MAX6675_CS1_PIN, error);

    std::cout << "  Decoded Temp: " << temp << " °C, Error: " << (error ? "TRUE" : "FALSE") << std::endl;
    assert(error);
    assert(temp == 0.0f);

    // Feed to live telemetry and broadcast
    g_temps[0] = 0.0f;
    g_sensorError[0] = true;
    broadcastLiveTelemetry();

    std::cout << "  Telemetry with Error: " << ws.lastTextAll << std::endl;
    assert(ws.lastTextAll.find("\"e1\":1") != std::string::npos);

    std::cout << "test_sensor_errors PASSED" << std::endl << std::endl;
}

int main() {
    test_live_telemetry();
    test_profiling_timeline_stream();
    test_max6675_decoding();
    test_sensor_errors();
    std::cout << "All extended MAX6675 feature tests PASSED successfully!" << std::endl;
    return 0;
}
