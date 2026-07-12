#include "definitions.h"
#include "dashboard_html.h"

#ifndef MOCK_TEST
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#endif

extern AsyncWebSocket ws;

// --- Stream Helpers ---
static inline void putU8(uint8_t *buf, size_t &off, uint8_t v) {
    buf[off++] = v;
}

static inline void putU16LE(uint8_t *buf, size_t &off, uint16_t v) {
    buf[off++] = (uint8_t)(v & 0xFF);
    buf[off++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void putU32LE(uint8_t *buf, size_t &off, uint32_t v) {
    buf[off++] = (uint8_t)(v & 0xFF);
    buf[off++] = (uint8_t)((v >> 8) & 0xFF);
    buf[off++] = (uint8_t)((v >> 16) & 0xFF);
    buf[off++] = (uint8_t)((v >> 24) & 0xFF);
}

// --- Frame Serialization and Broadcast ---
void sendFramePacket(bool timeoutFlag) {
    if (ws.count() == 0) return;

    // Allocate packet buffer
    uint8_t packet[18 + (CORE_COUNT * MAX_EVENTS_PER_CORE * 6)];
    size_t p = 0;

    // Snapshot counts and data quickly inside a critical section
    uint8_t c0, c1;
    EventRec events0[MAX_EVENTS_PER_CORE];
    EventRec events1[MAX_EVENTS_PER_CORE];

    portENTER_CRITICAL(&g_mux);
    c0 = (g_coreBuf[0].count > MAX_EVENTS_PER_CORE) ? MAX_EVENTS_PER_CORE : g_coreBuf[0].count;
    c1 = (g_coreBuf[1].count > MAX_EVENTS_PER_CORE) ? MAX_EVENTS_PER_CORE : g_coreBuf[1].count;
    memcpy(events0, (const void*)g_coreBuf[0].events, c0 * sizeof(EventRec));
    memcpy(events1, (const void*)g_coreBuf[1].events, c1 * sizeof(EventRec));
    portEXIT_CRITICAL(&g_mux);

    // Build the Header
    putU16LE(packet, p, 0x5450); // "TP"
    putU8(packet, p, 1);          // version
    putU8(packet, p, CORE_COUNT); // cores
    putU32LE(packet, p, g_frameSeq);
    putU32LE(packet, p, g_frameStartUs);
    putU16LE(packet, p, (uint16_t)FRAME_PERIOD_US);
    putU8(packet, p, c0);
    putU8(packet, p, c1);

    uint8_t flags = 0;
    if (timeoutFlag) flags |= 0x01;
    if (c0 >= MAX_EVENTS_PER_CORE || c1 >= MAX_EVENTS_PER_CORE) flags |= 0x02;
    putU8(packet, p, flags);
    putU8(packet, p, 0);

    // Serialize Core 0
    for (uint8_t i = 0; i < c0; i++) {
        putU8(packet, p, events0[i].taskId);
        putU8(packet, p, events0[i].flags);
        putU16LE(packet, p, events0[i].startUs);
        putU16LE(packet, p, events0[i].durUs);
    }

    // Serialize Core 1
    for (uint8_t i = 0; i < c1; i++) {
        putU8(packet, p, events1[i].taskId);
        putU8(packet, p, events1[i].flags);
        putU16LE(packet, p, events1[i].startUs);
        putU16LE(packet, p, events1[i].durUs);
    }

    // Broadcast to active, non-congested WS clients
    for (auto & client : ws.getClients()) {
        if (client.status() == WS_CONNECTED) {
            if (client.queueLen() < 4) {
                client.binary(packet, p);
            }
        }
    }
}

// --- Live Temperature Telemetry (JSON format) ---
void broadcastLiveTelemetry() {
    if (ws.count() == 0) return;

    float temps[4];
    bool errs[4];

    WEB_LOCK();
    for (int i = 0; i < 4; i++) {
        temps[i] = g_temps[i];
        errs[i] = g_sensorError[i];
    }
    WEB_UNLOCK();

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"t1\":%.2f,\"t2\":%.2f,\"t3\":%.2f,\"t4\":%.2f,\"e1\":%d,\"e2\":%d,\"e3\":%d,\"e4\":%d}",
        temps[0], temps[1], temps[2], temps[3],
        errs[0] ? 1 : 0, errs[1] ? 1 : 0, errs[2] ? 1 : 0, errs[3] ? 1 : 0);

    ws.textAll(buf);
}

// --- Serve Root Directory ---
void handleRoot(AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", INDEX_HTML);
    response->addHeader("Connection", "close");
    request->send(response);
}

// --- WebSocket Events ---
void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS Client connected [%u]\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS Client disconnected [%u]\n", client->id());
    }
}
