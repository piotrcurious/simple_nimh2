#ifndef DUMMY_ESP32_H
#define DUMMY_ESP32_H

#include <stdint.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdarg.h>
#include <cmath>
#include <algorithm>
#include <map>

// FreeRTOS dummies
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
#define xTaskCreate(a,b,c,d,e,f)
#define xTaskCreatePinnedToCore(a,b,c,d,e,f,g)
inline void vTaskDelay(uint32_t t) {}
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (SemaphoreHandle_t)1; }
#define xSemaphoreTake(s,t) (true)
#define xSemaphoreGive(s)
#define pdTRUE true
#define portMAX_DELAY 0xFFFFFFFF

// ADC dummies
typedef enum { ADC_ATTEN_DB_11 } adc_atten_t;
typedef struct { int dummy; } esp_adc_cal_characteristics_t;
struct AdcSnapshot {
    uint64_t sum;
    uint32_t count;
};
#define ADC_CH_COUNT 4

// SHT4x dummies
typedef enum { SHT4X_HIGH_PRECISION } sht4x_precision_t;
typedef enum { SHT4X_NO_HEATER } sht4x_heater_t;
struct Adafruit_SHT4x {
    void begin(void*) {}
    void setPrecision(sht4x_precision_t p) {}
    void setHeater(sht4x_heater_t h) {}
};

// Arduino dummies
#define PROGMEM
#define PSTR(s) s
#define R(x) x
#define F(x) x
#define OUTPUT 0
#define INPUT 1
#define HIGH 1
#define LOW 0

struct String : std::string {
    String() : std::string() {}
    String(const char* s) : std::string(s ? s : "") {}
    String(const std::string& s) : std::string(s) {}
    String(int v) : std::string(std::to_string(v)) {}
    String(unsigned int v) : std::string(std::to_string(v)) {}
    String(long v) : std::string(std::to_string(v)) {}
    String(unsigned long v) : std::string(std::to_string(v)) {}
    String(float v, int p = 2) : std::string() {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", p, (double)v);
        this->assign(buf);
    }
    String(double v, int p = 2) : std::string() {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", p, v);
        this->assign(buf);
    }
    void reserve(size_t n) { std::string::reserve(n); }
    String operator+(const String& other) const { return String((std::string)*this + (std::string)other); }
    String operator+(const char* other) const { return String((std::string)*this + std::string(other ? other : "")); }
    String& operator+=(const String& other) { std::string::operator+=(other); return *this; }
    String& operator+=(const char* other) { std::string::operator+=(other ? other : ""); return *this; }
    const char* c_str() const { return std::string::c_str(); }
    int find(const char* s) const {
        size_t pos = std::string::find(s);
        return (pos == std::string::npos) ? -1 : (int)pos;
    }
};

extern unsigned long mock_millis;
inline unsigned long millis() { return mock_millis; }

// Mock Serial with printf
struct MockSerial {
    void println(const char* s) { std::cout << (s?s:"") << std::endl; }
    void println(std::string s) { std::cout << s << std::endl; }
    void println(double v) { std::cout << v << std::endl; }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
    void print(const char* s) { std::cout << (s?s:""); }
    void print(double v) { std::cout << v; }
};
extern MockSerial Serial;

struct WebServer {
    int lastResponseCode;
    String lastResponseType;
    String lastResponseContent;
    std::map<String, String> args;

    void send(int code, const char* type, String content) {
        lastResponseCode = code;
        lastResponseType = type;
        lastResponseContent = content;
    }
    String arg(const String& name) {
        if (args.count(name)) return args[name];
        return "";
    }
    void on(const char* path, void (*handler)()) {}
    void begin() {}
    void handleClient() {}
};

inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) {}
inline int analogRead(int pin) { return 0; }
inline void analogWrite(int pin, int val) {}
inline void analogWriteResolution(int pin, int res) {}
inline void analogWriteFrequency(int pin, int freq) {}
inline void analogWriteFrequency(int freq) {}

#endif
