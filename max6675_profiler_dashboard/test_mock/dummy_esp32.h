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
#include <cstring>
#include <functional>

// Forward declarations for AsyncWebServer dummies
struct AsyncWebServerResponse;

// FreeRTOS dummies
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
#define xTaskCreate(a,b,c,d,e,f)
#define xTaskCreatePinnedToCore(a,b,c,d,e,f,g)
inline void vTaskDelay(uint32_t t) {}
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (SemaphoreHandle_t)1; }
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutex() { return (SemaphoreHandle_t)1; }
#define xSemaphoreTake(s,t) (true)
#define xSemaphoreGive(s)
#define xSemaphoreTakeRecursive(s,t) (true)
#define xSemaphoreGiveRecursive(s)
#define pdTRUE true
#define portMAX_DELAY 0xFFFFFFFF

typedef struct {
    int dummy;
} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
inline void portENTER_CRITICAL(portMUX_TYPE* mux) {}
inline void portEXIT_CRITICAL(portMUX_TYPE* mux) {}

// ESP32 timer mock
inline uint64_t esp_timer_get_time() { return 100000; }

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
    void begin(uint32_t baud) {}
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

// AsyncWebServer Dummies
enum AwsEventType {
    WS_EVT_CONNECT,
    WS_EVT_DISCONNECT,
    WS_EVT_PONG,
    WS_EVT_ERROR,
    WS_EVT_DATA
};

enum AwsFrameType {
    WS_CONTINUATION = 0x00,
    WS_TEXT = 0x01,
    WS_BINARY = 0x02,
    WS_DISCONNECT = 0x08,
    WS_PING = 0x09,
    WS_PONG = 0x0a
};

struct AwsFrameInfo {
    bool final;
    uint8_t opcode;
    bool masked;
    uint64_t index;
    uint64_t len;
    uint32_t num;
};

enum WS_STATUS {
    WS_CONNECTED
};

struct AsyncWebSocketClient {
    uint32_t id() { return 1; }
    std::vector<uint8_t> lastBinary;
    std::string lastText;
    WS_STATUS status() { return WS_CONNECTED; }
    size_t queueLen() { return 0; }
    void binary(const uint8_t* data, size_t len) {
        lastBinary.assign(data, data + len);
    }
    void text(const char* data) {
        lastText = data ? data : "";
    }
};

struct AsyncWebSocket {
    const char* _url;
    void (*_eventCallback)(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) = nullptr;
    std::vector<uint8_t> lastBinaryAll;
    std::string lastTextAll;
    std::vector<AsyncWebSocketClient> _clients;
    AsyncWebSocketClient _mockClient;
    AsyncWebSocket(const char* url) : _url(url) {
        _clients.push_back(_mockClient);
    }
    void onEvent(void (*cb)(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)) {
        _eventCallback = cb;
    }
    size_t count() { return 1; }
    std::vector<AsyncWebSocketClient>& getClients() { return _clients; }
    void binaryAll(const uint8_t* data, size_t len) {
        lastBinaryAll.assign(data, data + len);
    }
    void textAll(const char* text) {
        lastTextAll = text ? text : "";
    }
    void cleanupClients() {}
};

typedef std::function<size_t(uint8_t*, size_t, size_t)> AwsChunkedDataCallback;

struct AsyncWebServerRequest {
    std::map<String, String> _args;
    String arg(const String& name) { return _args.count(name) ? _args[name] : ""; }
    void send(int code, const char* type, String content);
    void send(AsyncWebServerResponse* response);
    AsyncWebServerResponse* beginResponse(int code, const char* type, const uint8_t* data, size_t size);
    AsyncWebServerResponse* beginResponse_P(int code, const char* type, const char* content);
    AsyncWebServerResponse* beginChunkedResponse(const char* type, AwsChunkedDataCallback cb);
};

struct AsyncWebServerResponse {
    void addHeader(const char* name, const char* value) {}
    String _content;
    bool _isChunked = false;
    AwsChunkedDataCallback _callback;
};

inline AsyncWebServerResponse* AsyncWebServerRequest::beginResponse(int code, const char* type, const uint8_t* data, size_t size) {
    AsyncWebServerResponse* r = new AsyncWebServerResponse();
    if (data && size) r->_content.assign((const char*)data, size);
    return r;
}
inline AsyncWebServerResponse* AsyncWebServerRequest::beginResponse_P(int code, const char* type, const char* content) {
    AsyncWebServerResponse* r = new AsyncWebServerResponse();
    if (content) r->_content.assign(content);
    return r;
}

inline AsyncWebServerResponse* AsyncWebServerRequest::beginChunkedResponse(const char* type, AwsChunkedDataCallback cb) {
    AsyncWebServerResponse* r = new AsyncWebServerResponse();
    r->_isChunked = true;
    r->_callback = cb;
    return r;
}

struct AsyncWebServer {
    AsyncWebServer(int port) {}
    void on(const char* url, int method, void (*handler)(AsyncWebServerRequest *request)) {}
    void addHandler(AsyncWebSocket* ws) {}
    void begin() {}
};

#define HTTP_GET 1

extern int (*mock_digitalRead_cb)(int pin);

inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) {}
inline int digitalRead(int pin) {
    if (mock_digitalRead_cb) return mock_digitalRead_cb(pin);
    return 0;
}
inline int analogRead(int pin) { return 0; }
inline void analogWrite(int pin, int val) {}
inline void analogWriteResolution(int pin, int res) {}
inline void analogWriteFrequency(int pin, int freq) {}
inline void analogWriteFrequency(int freq) {}
inline void delayMicroseconds(uint32_t us) {}
inline void delay(uint32_t ms) {}

#endif
