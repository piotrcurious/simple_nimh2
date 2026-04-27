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

#define CONTENT_LENGTH_UNKNOWN -1

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

struct AsyncWebSocketClient {
    uint32_t id() { return 1; }
    void binary(const uint8_t* data, size_t len) {}
    void text(const char* data) {}
};

struct AsyncWebSocket {
    const char* _url;
    void (*_eventCallback)(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) = nullptr;
    std::vector<uint8_t> lastBinaryAll;
    AsyncWebSocket(const char* url) : _url(url) {}
    void onEvent(void (*cb)(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)) {
        _eventCallback = cb;
    }
    size_t count() { return 1; }
    void binaryAll(const uint8_t* data, size_t len) {
        lastBinaryAll.assign(data, data + len);
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

typedef enum {
    WStype_ERROR,
    WStype_DISCONNECTED,
    WStype_CONNECTED,
    WStype_TEXT,
    WStype_BIN,
    WStype_FRAGMENT_TEXT_START,
    WStype_FRAGMENT_BIN_START,
    WStype_FRAGMENT,
    WStype_FRAGMENT_FIN,
    WStype_PING,
    WStype_PONG,
} WStype_t;

struct IPAddress {
    uint8_t _ip[4];
    uint8_t operator[](int i) const { return _ip[i]; }
};

struct WebSocketsServer {
    void (*_eventCallback)(uint8_t num, WStype_t type, uint8_t * payload, size_t length) = nullptr;
    std::vector<uint8_t> lastSentData;
    std::vector<uint8_t> lastBroadcastData;
    int _connectedClients = 0;

    WebSocketsServer(int port) {}
    void begin() {}
    void loop() {}
    void onEvent(void (*cb)(uint8_t num, WStype_t type, uint8_t * payload, size_t length)) {
        _eventCallback = cb;
    }
    void sendBIN(uint8_t num, const uint8_t * payload, size_t length) {
        lastSentData.assign(payload, payload + length);
    }
    void broadcastBIN(const uint8_t * payload, size_t length) {
        lastBroadcastData.assign(payload, payload + length);
    }
    int connectedClients() { return _connectedClients; }
    IPAddress remoteIP(uint8_t num) { return {{192, 168, 4, 2}}; }

    // Mock helpers
    void mockConnect(uint8_t num) {
        _connectedClients++;
        if (_eventCallback) _eventCallback(num, WStype_CONNECTED, (uint8_t*)"/", 1);
    }
    void mockDisconnect(uint8_t num) {
        if (_connectedClients > 0) _connectedClients--;
        if (_eventCallback) _eventCallback(num, WStype_DISCONNECTED, nullptr, 0);
    }
    void mockReceiveText(uint8_t num, const char* text) {
        if (_eventCallback) _eventCallback(num, WStype_TEXT, (uint8_t*)text, strlen(text));
    }
};

struct MockClient {
    String* output;
    void write(const uint8_t* buf, size_t size) {
        if (output) output->append((const char*)buf, size);
    }
};

struct WebServer {
    int lastResponseCode;
    String lastResponseType;
    String lastResponseContent;
    std::map<String, String> args;
    MockClient _mockClient;

    WebServer() { _mockClient.output = &lastResponseContent; }

    MockClient& client() { return _mockClient; }

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
    void setContentLength(int len) {}
    void sendHeader(const char* name, const char* value) {}
    void sendContent(const String& content) { lastResponseContent += content; }
    void sendContent(const char* data, size_t size) { lastResponseContent.append(data, size); }
};

inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) {}
inline int analogRead(int pin) { return 0; }
inline void analogWrite(int pin, int val) {}
inline void analogWriteResolution(int pin, int res) {}
inline void analogWriteFrequency(int pin, int freq) {}
inline void analogWriteFrequency(int freq) {}

#endif
