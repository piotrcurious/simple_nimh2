#ifndef WEBSERVER_H
#define WEBSERVER_H
#include "Arduino.h"
struct WebServer {
    void send(int code, const char* type, String content) {}
    String arg(const char* name) { return ""; }
    void on(const char* path, void (*handler)()) {}
    void begin() {}
    void handleClient() {}
};
#endif
