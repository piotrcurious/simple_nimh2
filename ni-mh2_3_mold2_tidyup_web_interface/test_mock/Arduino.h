#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <chrono>
#include <thread>
#include <stdint.h>
#include <stdarg.h>

#define PROGMEM
#define PSTR(s) s

typedef std::string String;

inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#define OUTPUT 1
inline void pinMode(int pin, int mode) {}
inline void analogWrite(int pin, int val) {}
inline void analogWriteResolution(int pin, int res) {}
inline void analogWriteFrequency(int pin, int freq) {}

class SerialMock {
public:
    void begin(int baud) {}
    void print(const String& s) { std::cout << s; }
    void print(double d, int p = 2) { std::cout << d; }
    void println(const String& s) { std::cout << s << std::endl; }
    void printf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
};
extern SerialMock Serial;

class WiFiMock {
public:
    void softAP(const char* ssid, const char* pass) {}
    String softAPIP() { return "192.168.4.1"; }
};
extern WiFiMock WiFi;

class WebServer {
public:
    WebServer(int port) {}
    typedef void (*Handler)();
    void on(const String& path, Handler h) { handlers[path] = h; }
    void begin() {}
    void handleClient() {}
    void send(int code, const String& type, const String& content) {
        std::cout << "HTTP " << code << " [" << type << "]: " << content.substr(0, 100) << "..." << std::endl;
    }
    String arg(const String& name) { return ""; }
private:
    std::map<String, Handler> handlers;
};

#define pdMS_TO_TICKS(ms) ms
typedef int TickType_t;
typedef void* TaskHandle_t;
inline void xTaskCreate(void (*f)(void*), const char* name, int stack, void* param, int prio, TaskHandle_t* h) {}
inline void vTaskDelay(int ticks) { delay(ticks); }

#define ADC_ATTEN_DB_11 0

// Mock FreeRTOS
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(int* mux) {}
inline void portEXIT_CRITICAL(int* mux) {}

#endif
