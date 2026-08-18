#include "definitions.h"
#include "home_screen.h"
#include "dashboard_html.h"

#ifndef MOCK_TEST
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#endif

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

#ifndef MOCK_TEST
extern AsyncWebServer server;
extern AsyncWebSocket ws;
#else
extern AsyncWebSocket ws;
#endif

extern void setAppState(AppState s);
extern void setBuildModelPhase(BuildModelPhase p);

#ifndef MOCK_TEST
inline AsyncWebSocketClient* resolve_client(AsyncWebSocketClient* c) { return c; }
inline AsyncWebSocketClient* resolve_client(AsyncWebSocketClient& c) { return &c; }
#endif

// Helper to check if a client is healthy enough to receive data
static bool canSend(AsyncWebSocketClient *client) {
    if (!client || client->status() != WS_CONNECTED) return false;
    return client->queueLen() < 16;
}

struct CborWriter {
    std::vector<uint8_t> data;
    bool overflow = false;

    bool ok() const { return !overflow; }

    void reserve(size_t n) {
        data.reserve(n);
    }
    void put(uint8_t b) {
        data.push_back(b);
    }
    void putBytes(const void* p, size_t n) {
        if (!p || n == 0) return;
        const uint8_t* b = static_cast<const uint8_t*>(p);
        data.insert(data.end(), b, b + n);
    }
    void addTypeVal(uint8_t major, uint64_t val) {
        if (val < 24) put((major << 5) | (uint8_t)val);
        else if (val <= 0xFF) { put((major << 5) | 24); put((uint8_t)val); }
        else if (val <= 0xFFFF) {
            put((major << 5) | 25);
            uint16_t v = (uint16_t)val;
            uint8_t tmp[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
            putBytes(tmp, 2);
        } else if (val <= 0xFFFFFFFFULL) {
            put((major << 5) | 26);
            uint32_t v = (uint32_t)val;
            uint8_t tmp[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
            putBytes(tmp, 4);
        } else {
            put((major << 5) | 27);
            for (int i = 7; i >= 0; i--) put((uint8_t)(val >> (8 * i)));
        }
    }
    void addUInt(uint64_t v) { addTypeVal(0, v); }
    void addInt(int64_t v) {
        if (v >= 0) addUInt((uint64_t)v);
        else addTypeVal(1, (uint64_t)(-(v + 1)));
    }
    void addFloat(float f) {
        if (!std::isfinite(f)) {
            addNull();
            return;
        }
        put(0xFA);
        uint32_t u;
        memcpy(&u, &f, sizeof(u));
        uint8_t tmp[4] = { (uint8_t)(u >> 24), (uint8_t)(u >> 16), (uint8_t)(u >> 8), (uint8_t)(u & 0xFF) };
        putBytes(tmp, 4);
    }
    void addNull() { put(0xF6); }
    void addText(const char* s) {
        if (!s) { addNull(); return; }
        size_t n = strlen(s);
        addTypeVal(3, n);
        putBytes(s, n);
    }
    void startArray(size_t n) { addTypeVal(4, n); }
    void startMap(size_t n) { addTypeVal(5, n); }
};

static void cborAddFloatArray(CborWriter& w, const float* arr, int len) {
    w.startArray(len);
    for (int i = 0; i < len; i++) {
        if (!std::isfinite(arr[i])) w.addNull();
        else w.addFloat(arr[i]);
    }
}

static void cborAddXYPairs(CborWriter& w, const float data[][2], int count) {
    w.startArray(count);
    for (int i = 0; i < count; i++) {
        w.startArray(2);
        w.addFloat(data[i][0]);
        w.addFloat(data[i][1]);
    }
}

struct StateSnapshot {
    AppState app;
    DisplayState display;
    uint8_t duty;
    float v;
    float i;
    float mah;
    float max_dt;
    BuildModelPhase phase;
    float offset;
    float noise;
};

static StateSnapshot getSnapshotState() {
    StateSnapshot s;
    WEB_LOCK();
    s.app = currentAppState;
    s.display = currentDisplayState;
    s.duty = dutyCycle;
    s.v = voltage_mv / 1000.0f;
    s.i = current_ma / 1000.0f;
    s.mah = (float)mAh_charged;
    s.max_dt = MAX_DIFF_TEMP;
    s.phase = buildModelPhase;
    s.offset = systemData.getCurrentZeroOffsetMv();
    s.noise = (float)noiseFloorMv;
    WEB_UNLOCK();
    return s;
}

static void appendCborState(CborWriter& w, const StateSnapshot& s) {
    w.startMap(10);
    w.addText("app");    w.addInt((int64_t)s.app);
    w.addText("display");w.addInt((int64_t)s.display);
    w.addText("duty");   w.addInt((int64_t)s.duty);
    w.addText("v");      w.addFloat(s.v);
    w.addText("i");      w.addFloat(s.i);
    w.addText("mah");    w.addFloat(s.mah);
    w.addText("max_dt"); w.addFloat(s.max_dt);
    w.addText("phase");  w.addInt((int64_t)s.phase);
    w.addText("offset"); w.addFloat(s.offset);
    w.addText("noise");  w.addFloat(s.noise);
}

struct HistorySnapshot {
    float t1[PLOT_WIDTH];
    float t2[PLOT_WIDTH];
    float td[PLOT_WIDTH];
    float v[PLOT_WIDTH];
    float i[PLOT_WIDTH];
};

static void getSnapshotHistory(HistorySnapshot& h) {
    WEB_LOCK();
    memcpy(h.t1, temp1_values, sizeof(h.t1));
    memcpy(h.t2, temp2_values, sizeof(h.t2));
    memcpy(h.td, diff_values, sizeof(h.td));
    memcpy(h.v, voltage_values, sizeof(h.v));
    memcpy(h.i, current_values, sizeof(h.i));
    WEB_UNLOCK();
}

static void appendCborHistory(CborWriter& w, const HistorySnapshot& h) {
    w.startMap(5);
    w.addText("t1"); cborAddFloatArray(w, h.t1, PLOT_WIDTH);
    w.addText("t2"); cborAddFloatArray(w, h.t2, PLOT_WIDTH);
    w.addText("td"); cborAddFloatArray(w, h.td, PLOT_WIDTH);
    w.addText("v");  cborAddFloatArray(w, h.v, PLOT_WIDTH);
    w.addText("i");  cborAddFloatArray(w, h.i, PLOT_WIDTH);
}

struct AmbientSnapshot {
    float t[PLOT_WIDTH];
    float h[PLOT_WIDTH];
    float d[PLOT_WIDTH];
};

static void getSnapshotAmbient(AmbientSnapshot& a) {
    WEB_LOCK();
    memcpy(a.t, homeScreen.temp_history, sizeof(a.t));
    memcpy(a.h, homeScreen.humidity_history, sizeof(a.h));
    memcpy(a.d, homeScreen.dew_point_history, sizeof(a.d));
    WEB_UNLOCK();
}

static void appendCborAmbient(CborWriter& w, const AmbientSnapshot& a) {
    w.startMap(3);
    w.addText("t"); cborAddFloatArray(w, a.t, PLOT_WIDTH);
    w.addText("h"); cborAddFloatArray(w, a.h, PLOT_WIDTH);
    w.addText("d"); cborAddFloatArray(w, a.d, PLOT_WIDTH);
}

struct IRSnapshot {
    float lu[MAX_RESISTANCE_POINTS][2];
    int luCount;
    float pairs[MAX_RESISTANCE_POINTS][2];
    int pairsCount;
};

static void getSnapshotIR(IRSnapshot& ir) {
    WEB_LOCK();
    ir.luCount = std::min(resistanceDataCount, (int)MAX_RESISTANCE_POINTS);
    if (ir.luCount > 0) {
        memcpy(ir.lu, internalResistanceData, sizeof(float) * 2 * ir.luCount);
    }
    ir.pairsCount = std::min(resistanceDataCountPairs, (int)MAX_RESISTANCE_POINTS);
    if (ir.pairsCount > 0) {
        memcpy(ir.pairs, internalResistanceDataPairs, sizeof(float) * 2 * ir.pairsCount);
    }
    WEB_UNLOCK();
}

static void appendCborIR(CborWriter& w, const IRSnapshot& ir) {
    w.startMap(2);
    w.addText("lu");    cborAddXYPairs(w, ir.lu, ir.luCount);
    w.addText("pairs"); cborAddXYPairs(w, ir.pairs, ir.pairsCount);
}

static void sendCborState(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;
    StateSnapshot state = getSnapshotState();
    CborWriter w;
    w.reserve(256);
    appendCborState(w, state);
    if (w.ok() && !w.data.empty()) {
        client->binary(w.data.data(), w.data.size());
    }
}

static void sendCborHistory(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;
    HistorySnapshot history;
    getSnapshotHistory(history);
    CborWriter w;
    w.reserve(PLOT_WIDTH * 5 * 8 + 32);
    appendCborHistory(w, history);
    if (w.ok() && !w.data.empty()) {
        client->binary(w.data.data(), w.data.size());
    }
}

static void sendCborAmbient(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;
    AmbientSnapshot ambient;
    getSnapshotAmbient(ambient);
    CborWriter w;
    w.reserve(PLOT_WIDTH * 3 * 8 + 32);
    appendCborAmbient(w, ambient);
    if (w.ok() && !w.data.empty()) {
        client->binary(w.data.data(), w.data.size());
    }
}

static void sendCborIR(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;
    IRSnapshot ir;
    getSnapshotIR(ir);
    CborWriter w;
    w.reserve(256 + (ir.luCount + ir.pairsCount) * 24);
    appendCborIR(w, ir);
    if (w.ok() && !w.data.empty()) {
        client->binary(w.data.data(), w.data.size());
    }
}

static void sendCborChargeLog(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;

    size_t total = 0;
    WEB_LOCK();
    total = chargeLog.size();
    WEB_UNLOCK();

    const size_t batchSize = 100;

    for (size_t i = 0; i < total; i += batchSize) {
        if (client->queueLen() > 8) {
            Serial.printf("Aborting CBOR log stream for client %u due to backed up queue.\n", client->id());
            break;
        }

        std::vector<ChargeLogData> batchEntries;
        batchEntries.reserve(batchSize);

        WEB_LOCK();
        size_t currentSize = chargeLog.size();
        for (size_t j = 0; j < batchSize && (i + j) < currentSize; j++) {
            batchEntries.push_back(chargeLog[i + j]);
        }
        WEB_UNLOCK();

        if (batchEntries.empty()) break;

        CborWriter w;
        w.reserve(batchEntries.size() * 150 + 64);
        w.startMap(3);
        w.addText("offset"); w.addUInt(i);
        w.addText("batch"); w.startArray(batchEntries.size());

        for (const auto& entry : batchEntries) {
            float td = entry.batteryTemperature - entry.ambientTemperature;
            float thresholdValue = entry.threshold;

            w.startMap(10);
            w.addText("t");    w.addUInt((uint64_t)entry.timestamp);
            w.addText("i");    w.addFloat(entry.current);
            w.addText("v");    w.addFloat(entry.voltage);
            w.addText("at");   w.addFloat(entry.ambientTemperature);
            w.addText("bt");   w.addFloat(entry.batteryTemperature);
            w.addText("d");    w.addInt((int64_t)entry.dutyCycle);
            w.addText("irlu"); w.addFloat(entry.internalResistanceLoadedUnloaded);
            w.addText("irp");  w.addFloat(entry.internalResistancePairs);
            w.addText("td");   w.addFloat(td);
            w.addText("th");   w.addFloat(thresholdValue);
        }
        w.addText("total"); w.addUInt(total);
        if (w.ok() && !w.data.empty()) {
            client->binary(w.data.data(), w.data.size());
        }
    }
}

static void sendCborRoot(AsyncWebSocketClient *client) {
    if (!canSend(client)) return;
    StateSnapshot state = getSnapshotState();
    AmbientSnapshot ambient;
    getSnapshotAmbient(ambient);

    CborWriter w;
    w.reserve(1024);
    w.startMap(2);
    w.addText("state");   appendCborState(w, state);
    w.addText("ambient"); appendCborAmbient(w, ambient);
    if (w.ok() && !w.data.empty()) {
        client->binary(w.data.data(), w.data.size());
    }
}

void handleData(AsyncWebServerRequest *request) {
    request->send(410, "text/plain", "API migrated to WebSockets. Use REQ_* commands.");
}

void handleRoot(AsyncWebServerRequest *request) {
    Serial.println("WEB: handleRoot");
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", INDEX_HTML);
    response->addHeader("Connection", "close");
    request->send(response);
}

static bool cmdMatch(const char* data, size_t len, const char* target) {
    size_t tlen = strlen(target);
    return (len == tlen && memcmp(data, target, len) == 0);
}

static void processCommandRaw(const char* data, size_t len, AsyncWebSocketClient *client = nullptr) {
    if (!data || len == 0) return;

    if (cmdMatch(data, len, "REQ_STATE")) { sendCborState(client); return; }
    if (cmdMatch(data, len, "REQ_HISTORY")) { sendCborHistory(client); return; }
    if (cmdMatch(data, len, "REQ_AMBIENT")) { sendCborAmbient(client); return; }
    if (cmdMatch(data, len, "REQ_CHARGELOG")) { sendCborChargeLog(client); return; }
    if (cmdMatch(data, len, "REQ_IR")) { sendCborIR(client); return; }

    if (cmdMatch(data, len, "charge")) {
        WEB_LOCK();
        resetAh = true;
        postModelAppState = APP_STATE_CHARGING;
        WEB_UNLOCK();
        setBuildModelPhase(BuildModelPhase::Idle);
        setAppState(APP_STATE_BUILDING_MODEL);
    } else if (cmdMatch(data, len, "ir")) {
        WEB_LOCK();
        isMeasuringResistance = true;
        if (currentModel.isModelBuilt) {
            currentIRState = IR_STATE_START;
            setAppState(APP_STATE_MEASURING_IR);
        } else {
            postModelAppState = APP_STATE_MEASURING_IR;
            setBuildModelPhase(BuildModelPhase::Idle);
            setAppState(APP_STATE_BUILDING_MODEL);
        }
        WEB_UNLOCK();
    } else if (cmdMatch(data, len, "reset")) {
        WEB_LOCK();
        resetAh = true;
        WEB_UNLOCK();
    } else if (cmdMatch(data, len, "stop")) {
        setAppState(APP_STATE_IDLE);
        applyDuty(0);
    }
}

static void processCommand(String cmd, AsyncWebSocketClient *client = nullptr) {
    processCommandRaw(cmd.c_str(), cmd.length(), client);
}

void handleCommand(AsyncWebServerRequest *request) {
    String cmd = request->arg("cmd");
    Serial.printf("DEBUG: Web command received: %s\n", cmd.c_str());
    processCommand(cmd);
    request->send(200, "text/plain", "OK");
}

void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS Client connected [%u]\n", client->id());
        sendCborState(client);
        sendCborAmbient(client);
    } else if (type == WS_EVT_DISCONNECT) {
        uint16_t reason = 0;
        if (arg != nullptr) {
            memcpy(&reason, arg, sizeof(reason));
        }
        Serial.printf("WS Disconnect [%u] Reason: %u\n", client->id(), reason);
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info && info->final && info->index == 0 && info->len == len) {
            if (info->opcode == WS_TEXT) {
                processCommandRaw((const char*)data, len, client);
            }
        }
    }
}

void broadcastLiveTelemetry() {
    static uint32_t lastBroadcast = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastBroadcast) < 1000U) return;
    lastBroadcast = now;

    if (ws.count() == 0) return;

    ws.cleanupClients();

    StateSnapshot state = getSnapshotState();

    CborWriter w;
    w.reserve(256);
    appendCborState(w, state);

    if (!w.ok() || w.data.empty()) return;

    for (auto & c : ws.getClients()) {
        AsyncWebSocketClient* client = resolve_client(c);
        if (client && client->status() == WS_CONNECTED) {
            if (client->queueLen() < 4) {
                client->binary(w.data.data(), w.data.size());
            }
        }
    }
}
