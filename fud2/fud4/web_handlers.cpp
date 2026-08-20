#include "definitions.h"
#include "home_screen.h"
#include "charging.h"
#include "dashboard_html.h"
#include "calibrate_html.h"

#ifndef MOCK_TEST
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#endif

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <memory>
#include <new>

#ifndef MOCK_TEST
extern AsyncWebServer server;
extern AsyncWebSocket ws;
#else
extern AsyncWebSocket ws;
#endif

extern void setAppState(AppState s);
extern void setBuildModelPhase(BuildModelPhase p);

template <typename T>
inline AsyncWebSocketClient* resolve_client(T* c) { return c; }

template <typename T>
inline AsyncWebSocketClient* resolve_client(T& c) { return &c; }

constexpr size_t WS_STATE_HIGH_WATER     = 8;
constexpr size_t WS_TELEMETRY_HIGH_WATER = 4;
constexpr size_t WS_LOG_HIGH_WATER       = 4;
constexpr size_t MAX_COMMAND_LENGTH      = 32;

// Helper to check if a client is healthy enough to receive data
static bool clientReadyForMessage(AsyncWebSocketClient *client, size_t maxQueueLen = WS_STATE_HIGH_WATER) {
    if (!client || client->status() != WS_CONNECTED) return false;
    return client->queueLen() < maxQueueLen;
}

class CborWriter {
private:
    uint8_t* buffer_;
    size_t capacity_;
    size_t pos_;
    bool overflow_;

public:
    CborWriter(uint8_t* buf, size_t cap)
        : buffer_(buf), capacity_(cap), pos_(0), overflow_(false) {}

    bool ok() const { return !overflow_; }
    size_t size() const { return pos_; }
    const uint8_t* data() const { return buffer_; }

    bool put(uint8_t b) {
        if (overflow_) return false;
        if (pos_ >= capacity_) {
            overflow_ = true;
            return false;
        }
        buffer_[pos_++] = b;
        return true;
    }

    bool putBytes(const void* p, size_t n) {
        if (overflow_) return false;
        if (!p || n == 0) return true;
        if (n > capacity_ - pos_) {
            overflow_ = true;
            return false;
        }
        memcpy(buffer_ + pos_, p, n);
        pos_ += n;
        return true;
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
    for (int i = 0; i < len && w.ok(); i++) {
        if (!std::isfinite(arr[i])) w.addNull();
        else w.addFloat(arr[i]);
    }
}

static void cborAddXYPairs(CborWriter& w, const float data[][2], int count) {
    if (!data || count <= 0) {
        w.startArray(0);
        return;
    }
    w.startArray(count);
    for (int i = 0; i < count && w.ok(); i++) {
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
    uint32_t free_heap;
    uint32_t min_free_heap;
    uint32_t max_alloc_heap;
    uint32_t total_heap;
    uint32_t chargelog_len;
    uint32_t thermal_hist_len;
    float shunt_r;
    float shunt_mv;
};

static StateSnapshot getSnapshotState() {
    StateSnapshot s;
#ifndef MOCK_TEST
    s.free_heap = ESP.getFreeHeap();
    s.min_free_heap = ESP.getMinFreeHeap();
    s.max_alloc_heap = ESP.getMaxAllocHeap();
    s.total_heap = ESP.getHeapSize();
#else
    s.free_heap = 180000;
    s.min_free_heap = 160000;
    s.max_alloc_heap = 110000;
    s.total_heap = 320000;
#endif

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
    s.chargelog_len = (uint32_t)chargeLog.size();
    s.thermal_hist_len = (uint32_t)s_thermalHistory.size();
    s.shunt_r = systemData.getShuntResistance();
    s.shunt_mv = systemData.getData().current_mv - systemData.getCurrentZeroOffsetMv();
    WEB_UNLOCK();
    return s;
}

static void appendCborState(CborWriter& w, const StateSnapshot& s) {
    w.startMap(18);
    w.addText("app");       w.addInt((int64_t)s.app);
    w.addText("display");   w.addInt((int64_t)s.display);
    w.addText("duty");      w.addInt((int64_t)s.duty);
    w.addText("v");         w.addFloat(s.v);
    w.addText("i");         w.addFloat(s.i);
    w.addText("mah");       w.addFloat(s.mah);
    w.addText("max_dt");    w.addFloat(s.max_dt);
    w.addText("phase");     w.addInt((int64_t)s.phase);
    w.addText("offset");    w.addFloat(s.offset);
    w.addText("noise");     w.addFloat(s.noise);
    w.addText("free_hp");   w.addUInt(s.free_heap);
    w.addText("min_hp");    w.addUInt(s.min_free_heap);
    w.addText("max_blk");   w.addUInt(s.max_alloc_heap);
    w.addText("tot_hp");    w.addUInt(s.total_heap);
    w.addText("log_len");   w.addUInt(s.chargelog_len);
    w.addText("th_len");    w.addUInt(s.thermal_hist_len);
    w.addText("shunt_r");   w.addFloat(s.shunt_r);
    w.addText("shunt_mv");  w.addFloat(s.shunt_mv);
}

static void sendCborState(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;
    StateSnapshot state = getSnapshotState();
    uint8_t buffer[256];
    CborWriter w(buffer, sizeof(buffer));
    appendCborState(w, state);
    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborHistory(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;

    size_t cap = PLOT_WIDTH * 5 * 5 + 64;
    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[cap]);
    if (!buf) return;

    std::unique_ptr<float[]> snap(new (std::nothrow) float[PLOT_WIDTH * 5]);
    if (!snap) return;

    float* snap_t1 = snap.get();
    float* snap_t2 = snap.get() + PLOT_WIDTH;
    float* snap_td = snap.get() + PLOT_WIDTH * 2;
    float* snap_v  = snap.get() + PLOT_WIDTH * 3;
    float* snap_i  = snap.get() + PLOT_WIDTH * 4;

    WEB_LOCK();
    memcpy(snap_t1, temp1_values, sizeof(float) * PLOT_WIDTH);
    memcpy(snap_t2, temp2_values, sizeof(float) * PLOT_WIDTH);
    memcpy(snap_td, diff_values, sizeof(float) * PLOT_WIDTH);
    memcpy(snap_v,  voltage_values, sizeof(float) * PLOT_WIDTH);
    memcpy(snap_i,  current_values, sizeof(float) * PLOT_WIDTH);
    WEB_UNLOCK();

    CborWriter w(buf.get(), cap);
    w.startMap(5);
    w.addText("t1"); cborAddFloatArray(w, snap_t1, PLOT_WIDTH);
    w.addText("t2"); cborAddFloatArray(w, snap_t2, PLOT_WIDTH);
    w.addText("td"); cborAddFloatArray(w, snap_td, PLOT_WIDTH);
    w.addText("v");  cborAddFloatArray(w, snap_v, PLOT_WIDTH);
    w.addText("i");  cborAddFloatArray(w, snap_i, PLOT_WIDTH);

    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborAmbient(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;

    size_t cap = PLOT_WIDTH * 2 * 5 + 64;
    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[cap]);
    if (!buf) return;

    float snap_t[PLOT_WIDTH];
    float snap_h[PLOT_WIDTH];

    WEB_LOCK();
    memcpy(snap_t, homeScreen.temp_history, sizeof(snap_t));
    memcpy(snap_h, homeScreen.humidity_history, sizeof(snap_h));
    WEB_UNLOCK();

    CborWriter w(buf.get(), cap);
    w.startMap(2);
    w.addText("t"); cborAddFloatArray(w, snap_t, PLOT_WIDTH);
    w.addText("h"); cborAddFloatArray(w, snap_h, PLOT_WIDTH);

    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborIR(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;

    size_t cap = 256 + (MAX_RESISTANCE_POINTS * 2) * 24;
    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[cap]);
    if (!buf) return;

    float snap_lu[MAX_RESISTANCE_POINTS][2];
    float snap_pairs[MAX_RESISTANCE_POINTS][2];
    int luCount = 0;
    int pairsCount = 0;

    WEB_LOCK();
    luCount = std::clamp(resistanceDataCount, 0, (int)MAX_RESISTANCE_POINTS);
    pairsCount = std::clamp(resistanceDataCountPairs, 0, (int)MAX_RESISTANCE_POINTS);
    if (luCount > 0) memcpy(snap_lu, internalResistanceData, luCount * sizeof(float) * 2);
    if (pairsCount > 0) memcpy(snap_pairs, internalResistanceDataPairs, pairsCount * sizeof(float) * 2);
    WEB_UNLOCK();

    CborWriter w(buf.get(), cap);
    w.startMap(2);
    w.addText("lu");    cborAddXYPairs(w, snap_lu, luCount);
    w.addText("pairs"); cborAddXYPairs(w, snap_pairs, pairsCount);

    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborChargeLog(AsyncWebSocketClient *client, size_t startOffset = 0) {
    if (!clientReadyForMessage(client, WS_LOG_HIGH_WATER)) return;

    constexpr size_t batchSize = 100;
    std::unique_ptr<ChargeLogData[]> batchEntries(new (std::nothrow) ChargeLogData[batchSize]);
    if (!batchEntries) return;

    size_t total = 0;
    size_t itemsInBatch = 0;
    uint32_t generation = 0;

    WEB_LOCK();
    total = chargeLog.size();
    generation = chargeLogGeneration;
    if (total > 0 && startOffset < total) {
        size_t batchEnd = std::min(total, startOffset + batchSize);
        for (size_t j = startOffset; j < batchEnd; j++) {
            batchEntries[itemsInBatch++] = chargeLog[j];
        }
    }
    WEB_UNLOCK();

    if (itemsInBatch == 0) return;

    constexpr size_t cap = batchSize * 60 + 128;
    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[cap]);
    if (!buf) return;

    uint8_t status = (startOffset + itemsInBatch >= total) ? 1 : 0; // 1 = complete, 0 = in-progress

    CborWriter w(buf.get(), cap);
    w.startMap(5);
    w.addText("offset"); w.addUInt(startOffset);
    w.addText("status"); w.addUInt(status);
    w.addText("gen");    w.addUInt(generation);
    w.addText("batch");  w.startArray(itemsInBatch);

    for (size_t k = 0; k < itemsInBatch; k++) {
        const auto& entry = batchEntries[k];

        w.startArray(8);
        w.addUInt((uint64_t)entry.timestamp);
        w.addFloat(entry.current);
        w.addFloat(entry.voltage);
        w.addFloat(entry.ambientTemperature);
        w.addFloat(entry.batteryTemperature);
        w.addFloat(entry.internalResistanceLoadedUnloaded);
        w.addFloat(entry.internalResistancePairs);
        w.addFloat(entry.threshold);
    }
    w.addText("total"); w.addUInt(total);
    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborRoot(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;

    size_t cap = PLOT_WIDTH * 2 * 5 + 384;
    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[cap]);
    if (!buf) return;

    StateSnapshot state = getSnapshotState();

    float snap_t[PLOT_WIDTH];
    float snap_h[PLOT_WIDTH];

    WEB_LOCK();
    memcpy(snap_t, homeScreen.temp_history, sizeof(snap_t));
    memcpy(snap_h, homeScreen.humidity_history, sizeof(snap_h));
    WEB_UNLOCK();

    CborWriter w(buf.get(), cap);
    w.startMap(2);
    w.addText("state");   appendCborState(w, state);

    w.addText("ambient");
    w.startMap(2);
    w.addText("t"); cborAddFloatArray(w, snap_t, PLOT_WIDTH);
    w.addText("h"); cborAddFloatArray(w, snap_h, PLOT_WIDTH);

    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
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

void handleCalibratePage(AsyncWebServerRequest *request) {
    Serial.println("WEB: handleCalibratePage");
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", CALIBRATE_HTML);
    response->addHeader("Connection", "close");
    request->send(response);
}

static bool parseSizeT(const char *s, size_t &out) {
    if (!s || !*s) return false;
    size_t value = 0;
    for (; *s; ++s) {
        if (*s < '0' || *s > '9') return false;
        const size_t digit = static_cast<size_t>(*s - '0');
        if (value > (SIZE_MAX - digit) / 10) return false;
        value = value * 10 + digit;
    }
    out = value;
    return true;
}

static bool cmdMatch(const char* data, size_t len, const char* target) {
    size_t tlen = strlen(target);
    return (len == tlen && memcmp(data, target, len) == 0);
}

static bool processCommandRaw(const char* data, size_t len, AsyncWebSocketClient *client = nullptr) {
    if (!data || len == 0 || len > MAX_COMMAND_LENGTH) return false;

    char cmdBuf[MAX_COMMAND_LENGTH + 1];
    memcpy(cmdBuf, data, len);
    cmdBuf[len] = '\0';

    if (cmdMatch(cmdBuf, len, "REQ_STATE")) {
        if (!client) return false;
        sendCborState(client);
        return true;
    }
    if (cmdMatch(cmdBuf, len, "REQ_HISTORY")) {
        if (!client) return false;
        sendCborHistory(client);
        return true;
    }
    if (cmdMatch(cmdBuf, len, "REQ_AMBIENT")) {
        if (!client) return false;
        sendCborAmbient(client);
        return true;
    }
    if (len >= 13 && memcmp(cmdBuf, "REQ_CHARGELOG", 13) == 0) {
        if (!client) return false;
        size_t startOffset = 0;
        if (len == 13) {
            sendCborChargeLog(client, 0);
            return true;
        } else if (cmdBuf[13] == ':') {
            if (!parseSizeT(cmdBuf + 14, startOffset)) return false;
            sendCborChargeLog(client, startOffset);
            return true;
        }
        return false;
    }
    if (cmdMatch(cmdBuf, len, "REQ_IR")) {
        if (!client) return false;
        sendCborIR(client);
        return true;
    }

    if (len >= 9 && (memcmp(cmdBuf, "SET_DUTY:", 9) == 0 || memcmp(cmdBuf, "set_duty:", 9) == 0)) {
        int val = atoi(cmdBuf + 9);
        val = std::clamp(val, 0, 255);
        setAppState(APP_STATE_IDLE);
        applyDuty(val);
        return true;
    }
    if (len >= 5 && (memcmp(cmdBuf, "duty:", 5) == 0 || memcmp(cmdBuf, "DUTY:", 5) == 0)) {
        int val = atoi(cmdBuf + 5);
        val = std::clamp(val, 0, 255);
        setAppState(APP_STATE_IDLE);
        applyDuty(val);
        return true;
    }
    if (len >= 12 && (memcmp(cmdBuf, "SET_SHUNT_R:", 12) == 0 || memcmp(cmdBuf, "set_shunt_r:", 12) == 0)) {
        float val = atof(cmdBuf + 12);
        if (std::isfinite(val) && val > 0.01f && val < 1000.0f) {
            systemData.setShuntResistance(val);
            return true;
        }
        return false;
    }
    if (len >= 13 && (memcmp(cmdBuf, "SAVE_SHUNT_R:", 13) == 0 || memcmp(cmdBuf, "save_shunt_r:", 13) == 0)) {
        float val = atof(cmdBuf + 13);
        if (std::isfinite(val) && val > 0.01f && val < 1000.0f) {
            systemData.saveShuntResistance(val);
            return true;
        }
        return false;
    }
    if (cmdMatch(cmdBuf, len, "SAVE_SHUNT_R") || cmdMatch(cmdBuf, len, "save_shunt_r")) {
        systemData.saveShuntResistance(systemData.getShuntResistance());
        return true;
    }

    if (cmdMatch(cmdBuf, len, "charge")) {
        WEB_LOCK();
        resetAh = true;
        postModelAppState = APP_STATE_CHARGING;
        WEB_UNLOCK();
        setBuildModelPhase(BuildModelPhase::Idle);
        setAppState(APP_STATE_BUILDING_MODEL);
        return true;
    } else if (cmdMatch(cmdBuf, len, "ir")) {
        bool modelBuilt = false;
        WEB_LOCK();
        isMeasuringResistance = true;
        modelBuilt = currentModel.isModelBuilt;
        if (modelBuilt) {
            currentIRState = IR_STATE_START;
        } else {
            postModelAppState = APP_STATE_MEASURING_IR;
        }
        WEB_UNLOCK();

        if (modelBuilt) {
            setAppState(APP_STATE_MEASURING_IR);
        } else {
            setBuildModelPhase(BuildModelPhase::Idle);
            setAppState(APP_STATE_BUILDING_MODEL);
        }
        return true;
    } else if (cmdMatch(cmdBuf, len, "reset")) {
        WEB_LOCK();
        resetAh = true;
        WEB_UNLOCK();
        return true;
    } else if (cmdMatch(cmdBuf, len, "stop")) {
        setAppState(APP_STATE_IDLE);
        applyDuty(0);
        return true;
    }

    return false;
}

void handleCommand(AsyncWebServerRequest *request) {
    String cmd = request->arg("cmd");
    Serial.printf("DEBUG: Web command received: %s\n", cmd.c_str());
    bool recognized = processCommandRaw(cmd.c_str(), cmd.length(), nullptr);
    if (recognized) {
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request: Unknown or unsupported command");
    }
}

void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS Client connected [%u]\n", client->id());
        sendCborRoot(client);
    } else if (type == WS_EVT_DISCONNECT) {
        uint16_t reason = 0;
        if (arg != nullptr) {
            memcpy(&reason, arg, sizeof(reason));
        }
        Serial.printf("WS Disconnect [%u] Reason: %u\n", client->id(), reason);
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info && info->final && info->index == 0 && info->len == len && len <= MAX_COMMAND_LENGTH) {
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

    ws.cleanupClients();
    if (ws.count() == 0) return;

    StateSnapshot state = getSnapshotState();

    uint8_t buffer[256];
    CborWriter w(buffer, sizeof(buffer));
    appendCborState(w, state);

    if (!w.ok() || w.size() == 0) return;

    for (auto & c : ws.getClients()) {
        AsyncWebSocketClient* client = resolve_client(c);
        if (client && client->status() == WS_CONNECTED) {
            if (client->queueLen() < WS_TELEMETRY_HIGH_WATER) {
                client->binary(w.data(), w.size());
            }
        }
    }
}
