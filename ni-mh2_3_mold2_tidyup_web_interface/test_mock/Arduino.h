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
#include <algorithm>
#include <sstream>
#include <iomanip>

#define PROGMEM
#define PSTR(s) s
#define FLASH_HELPER(s) s

class String : public std::string {
public:
    String() : std::string("") {}
    String(const char* s) : std::string(s ? s : "") {}
    String(const std::string& s) : std::string(s) {}
    String(int val) : std::string(std::to_string(val)) {}
    String(long val) : std::string(std::to_string(val)) {}
    String(unsigned int val) : std::string(std::to_string(val)) {}
    String(unsigned long val) : std::string(std::to_string(val)) {}
    String(float val, int prec = 2) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(prec) << val;
        *this = ss.str();
    }
    String(double val, int prec = 2) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(prec) << val;
        *this = ss.str();
    }

    String operator+(const String& other) const { return String((std::string)*this + (std::string)other); }
    String operator+(const char* other) const { return String((std::string)*this + std::string(other ? other : "")); }
    void operator+=(const String& other) { this->append(other); }
    void operator+=(const char* other) { if(other) this->append(other); }

    float toFloat() const { return std::stof(*this); }
    int toInt() const { return std::stoi(*this); }
};

extern unsigned long mock_millis;
inline unsigned long millis() { return mock_millis; }
inline void delay(unsigned long ms) { mock_millis += ms; }

#define OUTPUT 1
#define INPUT 0
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
    void println() { std::cout << std::endl; }
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
    String softAPIP() { return String("192.168.4.1"); }
};
extern WiFiMock WiFi;

class WebServer {
public:
    WebServer(int port) {}
    typedef void (*Handler)();
    void on(const String& path, Handler h) { handlers[(std::string)path] = h; }
    void begin() {}
    void handleClient() {}
    void send(int code, const String& type, const String& content) {}
    String arg(const String& name) {
        if (args.count((std::string)name)) return String(args[(std::string)name]);
        return String("");
    }
    std::map<std::string, std::string> args;
    std::map<std::string, Handler> handlers;
};

#define pdMS_TO_TICKS(ms) ms
typedef int TickType_t;
typedef void* TaskHandle_t;
inline void xTaskCreate(void (*f)(void*), const char* name, int stack, void* param, int prio, TaskHandle_t* h) {}
inline void vTaskDelay(int ticks) { mock_millis += ticks; }

#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(int* mux) {}
inline void portEXIT_CRITICAL(int* mux) {}

#define constrain(amt,low,high) std::clamp(amt,low,high)

typedef void* SemaphoreHandle_t;

// Provide global names for logic files
using std::max;
using std::min;
using std::isnan;
using std::isfinite;
using std::pow;
using std::sqrt;
using std::fabs;
using std::fmod;
using std::exp;
using std::log;

namespace Eigen {
    struct VectorXd {
        std::vector<double> coeffs;
        VectorXd() {}
        VectorXd(int size) : coeffs(size, 0.0) { if(size==0) coeffs.resize(4,0); }
        int size() const { return (int)coeffs.size(); }
        double& operator()(int i) { if(i >= (int)coeffs.size()) coeffs.resize(i+1, 0); return coeffs[i]; }
        double operator()(int i) const { return coeffs[i]; }

        struct Householder {
            VectorXd solve(const VectorXd& b) { return VectorXd(4); }
        };
        Householder householderQr() { return Householder(); }
    };

    struct MatrixXd {
        int r, c;
        std::vector<double> data;
        MatrixXd(int rows, int cols) : r(rows), c(cols), data(rows * cols, 0.0) {}
        double& operator()(int i, int j) { return data[i * c + j]; }

        struct QR {
            VectorXd solve(const struct VectorXd& b) {
                return VectorXd(4);
            }
        };
        QR householderQr() { return QR(); }
    };
}

#endif
