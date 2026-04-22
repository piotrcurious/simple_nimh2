#ifndef DUMMY_ESP32_H
#define DUMMY_ESP32_H

#include <stdint.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdarg.h>

// FreeRTOS dummies
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
#define xTaskCreate(a,b,c,d,e,f)

// ADC dummies
typedef enum { ADC_ATTEN_DB_11 } adc_atten_t;
typedef struct { int dummy; } esp_adc_cal_characteristics_t;

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
#define R(x) x
#define F(x) x
#define OUTPUT 0
typedef std::string String;

extern unsigned long mock_millis;
inline unsigned long millis() { return mock_millis; }

// Mock Serial with printf
struct MockSerial {
    void println(const char* s) { std::cout << s << std::endl; }
    void println(std::string s) { std::cout << s << std::endl; }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
};
extern MockSerial Serial;

#endif
