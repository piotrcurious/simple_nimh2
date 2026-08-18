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
    std::vector<uint8_t>* vec_owned_;

public:
    CborWriter(uint8_t* buf, size_t cap)
        : buffer_(buf), capacity_(cap), pos_(0), overflow_(false), vec_owned_(nullptr) {}

    CborWriter(std::vector<uint8_t>& vec)
        : buffer_(nullptr), capacity_(0), pos_(0), overflow_(false), vec_owned_(&vec) {}

    bool ok() const { return !overflow_; }
    size_t size() const { return vec_owned_ ? vec_owned_->size() : pos_; }
    const uint8_t* data() const { return vec_owned_ ? vec_owned_->data() : buffer_; }

    bool put(uint8_t b) {
        if (overflow_) return false;
        if (vec_owned_) {
            vec_owned_->push_back(b);
            return true;
        }
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
        if (vec_owned_) {
            const uint8_t* b = static_cast<const uint8_t*>(p);
            vec_owned_->insert(vec_owned_->end(), b, b + n);
            return true;
        }
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
    for (int i = 0; i < len; i++) {
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

static HistorySnapshot s_historySnapshot;

static void getSnapshotHistory() {
    WEB_LOCK();
    memcpy(s_historySnapshot.t1, temp1_values, sizeof(s_historySnapshot.t1));
    memcpy(s_historySnapshot.t2, temp2_values, sizeof(s_historySnapshot.t2));
    memcpy(s_historySnapshot.td, diff_values, sizeof(s_historySnapshot.td));
    memcpy(s_historySnapshot.v, voltage_values, sizeof(s_historySnapshot.v));
    memcpy(s_historySnapshot.i, current_values, sizeof(s_historySnapshot.i));
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

static AmbientSnapshot s_ambientSnapshot;

static void getSnapshotAmbient() {
    WEB_LOCK();
    memcpy(s_ambientSnapshot.t, homeScreen.temp_history, sizeof(s_ambientSnapshot.t));
    memcpy(s_ambientSnapshot.h, homeScreen.humidity_history, sizeof(s_ambientSnapshot.h));
    memcpy(s_ambientSnapshot.d, homeScreen.dew_point_history, sizeof(s_ambientSnapshot.d));
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

static IRSnapshot s_irSnapshot;

static void getSnapshotIR() {
    WEB_LOCK();
    s_irSnapshot.luCount = std::clamp(resistanceDataCount, 0, (int)MAX_RESISTANCE_POINTS);
    if (s_irSnapshot.luCount > 0) {
        memcpy(s_irSnapshot.lu, internalResistanceData, sizeof(float) * 2 * s_irSnapshot.luCount);
    }
    s_irSnapshot.pairsCount = std::clamp(resistanceDataCountPairs, 0, (int)MAX_RESISTANCE_POINTS);
    if (s_irSnapshot.pairsCount > 0) {
        memcpy(s_irSnapshot.pairs, internalResistanceDataPairs, sizeof(float) * 2 * s_irSnapshot.pairsCount);
    }
    WEB_UNLOCK();
}

static void appendCborIR(CborWriter& w, const IRSnapshot& ir) {
    w.startMap(2);
    w.addText("lu");    cborAddXYPairs(w, ir.lu, ir.luCount);
    w.addText("pairs"); cborAddXYPairs(w, ir.pairs, ir.pairsCount);
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
    getSnapshotHistory();
    std::vector<uint8_t> vec;
    vec.reserve(PLOT_WIDTH * 5 * 8 + 32);
    CborWriter w(vec);
    appendCborHistory(w, s_historySnapshot);
    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborAmbient(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;
    getSnapshotAmbient();
    std::vector<uint8_t> vec;
    vec.reserve(PLOT_WIDTH * 3 * 8 + 32);
    CborWriter w(vec);
    appendCborAmbient(w, s_ambientSnapshot);
    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborIR(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;
    getSnapshotIR();
    std::vector<uint8_t> vec;
    vec.reserve(256 + (s_irSnapshot.luCount + s_irSnapshot.pairsCount) * 24);
    CborWriter w(vec);
    appendCborIR(w, s_irSnapshot);
    if (w.ok() && w.size() > 0) {
        client->binary(w.data(), w.size());
    }
}

static void sendCborChargeLog(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_LOG_HIGH_WATER)) return;

    size_t total = 0;
    WEB_LOCK();
    total = chargeLog.size();
    WEB_UNLOCK();

    if (total == 0) return;

    const size_t batchSize = 100;

    for (size_t i = 0; i < total; i += batchSize) {
        if (client->queueLen() >= WS_LOG_HIGH_WATER) {
            Serial.printf("Aborting CBOR log stream for client %u due to backed up queue.\n", client->id());
            break;
        }

        std::vector<ChargeLogData> batchEntries;
        batchEntries.reserve(batchSize);

        WEB_LOCK();
        size_t currentSize = chargeLog.size();
        size_t batchTotal = std::min(total, currentSize);
        for (size_t j = 0; j < batchSize && (i + j) < batchTotal; j++) {
            batchEntries.push_back(chargeLog[i + j]);
        }
        WEB_UNLOCK();

        if (batchEntries.empty()) break;

        std::vector<uint8_t> vec;
        vec.reserve(batchEntries.size() * 150 + 64);
        CborWriter w(vec);
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
        if (w.ok() && w.size() > 0) {
            client->binary(w.data(), w.size());
        }
    }
}

static void sendCborRoot(AsyncWebSocketClient *client) {
    if (!clientReadyForMessage(client, WS_STATE_HIGH_WATER)) return;
    StateSnapshot state = getSnapshotState();
    getSnapshotAmbient();

    std::vector<uint8_t> vec;
    vec.reserve(PLOT_WIDTH * 3 * 8 + 384);
    CborWriter w(vec);
    w.startMap(2);
    w.addText("state");   appendCborState(w, state);
    w.addText("ambient"); appendCborAmbient(w, s_ambientSnapshot);
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

static bool cmdMatch(const char* data, size_t len, const char* target) {
    size_t tlen = strlen(target);
    return (len == tlen && memcmp(data, target, len) == 0);
}

static void processCommandRaw(const char* data, size_t len, AsyncWebSocketClient *client = nullptr) {
    if (!data || len == 0 || len > MAX_COMMAND_LENGTH) return;

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
