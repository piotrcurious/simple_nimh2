#include "definitions.h"
#include "home_screen.h"
#include "dashboard_html.h"

#ifndef MOCK_TEST
#include <WiFi.h>
#include <NetworkClient.h>
#include <WebServer.h>
#endif

#include <cmath>
#include <cstring>
#include <vector>

extern WebServer server;
extern void setAppState(AppState s);
extern void setBuildModelPhase(BuildModelPhase p);

/**
 * HELPER: Streams a float array as JSON directly to the client.
 * Uses a small buffer to group entries and reduce network overhead.
 */
static void streamFloatArrayJSON(const char* name, const float* arr, int len, bool isLast = false) {
    String s = "\"";
    s += name;
    s += "\":[";
    server.sendContent(s);
    s = "";
    for (int i = 0; i < len; i++) {
        if (std::isnan(arr[i])) s += "null";
        else {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f", arr[i]);
            s += buf;
        }
        if (i < len - 1) s += ",";
        if (s.length() > 400) {
            server.sendContent(s);
            s = "";
        }
    }
    s += isLast ? "]" : "],";
    server.sendContent(s);
}

String getJsonState() {
    char buf[256];
    WEB_LOCK();
    snprintf(buf, sizeof(buf),
        "{\"app\":%d,\"display\":%d,\"duty\":%d,\"v\":%.3f,\"i\":%.3f,\"mah\":%.3f,\"max_dt\":%.2f,\"phase\":%d,\"offset\":%.2f,\"noise\":%.2f}",
        (int)currentAppState, (int)currentDisplayState, (int)dutyCycle,
        voltage_mv / 1000.0f, current_ma / 1000.0f, (float)mAh_charged,
        MAX_DIFF_TEMP, (int)buildModelPhase, systemData.getCurrentZeroOffsetMv(), (float)noiseFloorMv);
    WEB_UNLOCK();
    return String(buf);
}

static void streamJsonHistory() {
    float t1[PLOT_WIDTH], t2[PLOT_WIDTH], td[PLOT_WIDTH], v[PLOT_WIDTH], i[PLOT_WIDTH];
    WEB_LOCK();
    memcpy(t1, temp1_values, sizeof(t1));
    memcpy(t2, temp2_values, sizeof(t2));
    memcpy(td, diff_values, sizeof(td));
    memcpy(v, voltage_values, sizeof(v));
    memcpy(i, current_values, sizeof(i));
    WEB_UNLOCK();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "{");
    streamFloatArrayJSON("t1", t1, PLOT_WIDTH);
    streamFloatArrayJSON("t2", t2, PLOT_WIDTH);
    streamFloatArrayJSON("td", td, PLOT_WIDTH);
    streamFloatArrayJSON("v", v, PLOT_WIDTH);
    streamFloatArrayJSON("i", i, PLOT_WIDTH, true);
    server.sendContent("}");
    server.sendContent("");
}

static void streamJsonAmbient() {
    float t[PLOT_WIDTH], h[PLOT_WIDTH], d[PLOT_WIDTH];
    WEB_LOCK();
    memcpy(t, homeScreen.temp_history, sizeof(t));
    memcpy(h, homeScreen.humidity_history, sizeof(h));
    memcpy(d, homeScreen.dew_point_history, sizeof(d));
    WEB_UNLOCK();

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "{");
    streamFloatArrayJSON("t", t, PLOT_WIDTH);
    streamFloatArrayJSON("h", h, PLOT_WIDTH);
    streamFloatArrayJSON("d", d, PLOT_WIDTH, true);
    server.sendContent("}");
    server.sendContent("");
}

struct CborWriter {
    std::vector<uint8_t> data;
    void reserve(size_t n) { data.reserve(n); }
    void put(uint8_t b) { data.push_back(b); }
    void putBytes(const void* p, size_t n) {
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
        else addTypeVal(1, (uint64_t)(-1 - v));
    }
    void addFloat(float f) {
        put(0xFA);
        uint32_t u;
        memcpy(&u, &f, sizeof(u));
        uint8_t tmp[4] = { (uint8_t)(u >> 24), (uint8_t)(u >> 16), (uint8_t)(u >> 8), (uint8_t)(u & 0xFF) };
        putBytes(tmp, 4);
    }
    void addNull() { put(0xF6); }
    void addText(const char* s) {
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
        if (std::isnan(arr[i])) w.addNull();
        else w.addFloat(arr[i]);
    }
}

static void cborAddXYPairs(CborWriter& w, float data[][2], int count) {
    w.startArray(count);
    for (int i = 0; i < count; i++) {
        w.startArray(2);
        w.addFloat(data[i][0]);
        w.addFloat(data[i][1]);
    }
}

static void appendCborState(CborWriter& w) {
    w.startMap(10);
    w.addText("app");    w.addInt((int64_t)currentAppState);
    w.addText("display");w.addInt((int64_t)currentDisplayState);
    w.addText("duty");   w.addInt((int64_t)dutyCycle);
    w.addText("v");      w.addFloat(voltage_mv / 1000.0f);
    w.addText("i");      w.addFloat(current_ma / 1000.0f);
    w.addText("mah");    w.addFloat((float)mAh_charged);
    w.addText("max_dt"); w.addFloat(MAX_DIFF_TEMP);
    w.addText("phase");  w.addInt((int64_t)buildModelPhase);
    w.addText("offset"); w.addFloat(systemData.getCurrentZeroOffsetMv());
    w.addText("noise");  w.addFloat((float)noiseFloorMv);
}

static void appendCborHistory(CborWriter& w) {
    w.startMap(5);
    w.addText("t1"); cborAddFloatArray(w, temp1_values, PLOT_WIDTH);
    w.addText("t2"); cborAddFloatArray(w, temp2_values, PLOT_WIDTH);
    w.addText("td"); cborAddFloatArray(w, diff_values, PLOT_WIDTH);
    w.addText("v");  cborAddFloatArray(w, voltage_values, PLOT_WIDTH);
    w.addText("i");  cborAddFloatArray(w, current_values, PLOT_WIDTH);
}

static void appendCborAmbient(CborWriter& w) {
    w.startMap(3);
    w.addText("t"); cborAddFloatArray(w, homeScreen.temp_history, PLOT_WIDTH);
    w.addText("h"); cborAddFloatArray(w, homeScreen.humidity_history, PLOT_WIDTH);
    w.addText("d"); cborAddFloatArray(w, homeScreen.dew_point_history, PLOT_WIDTH);
}

static void appendCborIR(CborWriter& w) {
    w.startMap(2);
    w.addText("lu");    cborAddXYPairs(w, internalResistanceData, resistanceDataCount);
    w.addText("pairs"); cborAddXYPairs(w, internalResistanceDataPairs, resistanceDataCountPairs);
}

static void sendBinaryResponse(const char* contentType, const std::vector<uint8_t>& payload) {
#ifndef MOCK_TEST
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.sendHeader(F("Connection"), F("close"));
    server.setContentLength(payload.size());
    server.send(200, contentType, "");
    if (!payload.empty()) {
        server.sendContent((const char*)payload.data(), payload.size());
    }
#endif
}

static void sendCborState() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(256);
    appendCborState(w);
    WEB_UNLOCK();
    sendBinaryResponse("application/cbor", w.data);
}

static void sendCborHistory() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(PLOT_WIDTH * 5 * 8 + 32);
    appendCborHistory(w);
    WEB_UNLOCK();
    sendBinaryResponse("application/cbor", w.data);
}

static void sendCborAmbient() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(PLOT_WIDTH * 3 * 8 + 32);
    appendCborAmbient(w);
    WEB_UNLOCK();
    sendBinaryResponse("application/cbor", w.data);
}

static void sendCborIR() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(256 + (resistanceDataCount + resistanceDataCountPairs) * 24);
    appendCborIR(w);
    WEB_UNLOCK();
    sendBinaryResponse("application/cbor", w.data);
}

static void sendCborChargeLog() {
#ifndef MOCK_TEST
    size_t total = 0;
    float currentRParam = 0.0f;
    {
        WEB_LOCK();
        total = chargeLog.size();
        currentRParam = regressedInternalResistancePairsIntercept;
        WEB_UNLOCK();
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/cbor", "");

    uint32_t lastTimestamp = 0;
    const size_t batchSize = 20;
    for (size_t i = 0; i < total; i += batchSize) {
        size_t currentBatchSize = std::min(batchSize, total - i);
        std::vector<ChargeLogData> batch;
        batch.reserve(currentBatchSize);

        WEB_LOCK();
        if (i < chargeLog.size()) {
            size_t end = std::min(i + currentBatchSize, chargeLog.size());
            for (size_t j = i; j < end; j++) {
                batch.push_back(chargeLog[j]);
            }
        }
        WEB_UNLOCK();

        if (batch.empty()) break;

        CborWriter w;
        w.reserve(batch.size() * 100);

        if (i == 0) w.put(0x9F); // Start indefinite array

        for (size_t j = 0; j < batch.size(); j++) {
            const auto& entry = batch[j];
            float td = entry.batteryTemperature - entry.ambientTemperature;
            float localEnergy = 0.0f;
            uint32_t prevTs = (i == 0 && j == 0) ? entry.timestamp : lastTimestamp;
            float estimatedDiff = estimateTempDiff(
                entry.voltage, entry.voltage, entry.current,
                currentRParam, entry.ambientTemperature,
                entry.timestamp, prevTs, entry.batteryTemperature,
                &localEnergy
            );
            float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            lastTimestamp = entry.timestamp;

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
        server.client().write(w.data.data(), w.data.size());
    }

    uint8_t stopByte = 0xFF; // Break for indefinite array
    server.client().write(&stopByte, 1);
    server.sendContent("");
#endif
}

static void sendCborRoot() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(512);
    w.startMap(2);
    w.addText("state");   appendCborState(w);
    w.addText("ambient"); appendCborAmbient(w);
    WEB_UNLOCK();
    sendBinaryResponse("application/cbor", w.data);
}

static void streamJsonChargeLog() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");
    server.sendContent("[");

    size_t total = 0;
    float currentRParam = 0.0f;
    {
        WEB_LOCK();
        total = chargeLog.size();
        currentRParam = regressedInternalResistancePairsIntercept;
        WEB_UNLOCK();
    }

    const size_t batchSize = 10;
    uint32_t lastTimestamp = 0;
    for (size_t i = 0; i < total; i += batchSize) {
        size_t currentBatch = std::min(batchSize, total - i);
        std::vector<ChargeLogData> batch;
        batch.reserve(currentBatch);

        WEB_LOCK();
        if (i < chargeLog.size()) {
            size_t end = std::min(i + currentBatch, chargeLog.size());
            for (size_t j = i; j < end; j++) {
                batch.push_back(chargeLog[j]);
            }
        }
        WEB_UNLOCK();

        if (batch.empty()) break;

        String chunk = "";
        chunk.reserve(batch.size() * 160);
        char buf[256];
        for (size_t j = 0; j < batch.size(); j++) {
            const auto& entry = batch[j];
            float td = entry.batteryTemperature - entry.ambientTemperature;
            float localEnergy = 0.0f;
            uint32_t prevTs = (i == 0 && j == 0) ? entry.timestamp : lastTimestamp;
            float estimatedDiff = estimateTempDiff(
                entry.voltage, entry.voltage, entry.current,
                currentRParam, entry.ambientTemperature,
                entry.timestamp, prevTs, entry.batteryTemperature,
                &localEnergy
            );
            float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            lastTimestamp = entry.timestamp;

            snprintf(buf, sizeof(buf),
                "{\"t\":%u,\"i\":%.3f,\"v\":%.3f,\"at\":%.2f,\"bt\":%.2f,\"d\":%d,\"irlu\":%.3f,\"irp\":%.3f,\"td\":%.2f,\"th\":%.2f}",
                (unsigned int)entry.timestamp, entry.current, entry.voltage, entry.ambientTemperature,
                entry.batteryTemperature, entry.dutyCycle, entry.internalResistanceLoadedUnloaded,
                entry.internalResistancePairs, td, thresholdValue
            );
            chunk += buf;
            if (i + j < total - 1) chunk += ",";
        }
        server.sendContent(chunk);
    }
    server.sendContent("]");
    server.sendContent("");
}

void handleData() {
    Serial.printf("WEB: handleData type=%s\n", server.arg("type").c_str());
    String type = server.arg("type");
    bool wantCbor = (server.arg("fmt") == "cbor");

    if (wantCbor) {
        if (type == "state") sendCborState();
        else if (type == "history") sendCborHistory();
        else if (type == "ambient") sendCborAmbient();
        else if (type == "chargelog") sendCborChargeLog();
        else if (type == "ir") sendCborIR();
        else sendCborRoot();
        return;
    }

    if (type == "state") server.send(200, "application/json", getJsonState());
    else if (type == "history") streamJsonHistory();
    else if (type == "ambient") streamJsonAmbient();
    else if (type == "chargelog") streamJsonChargeLog();
    else if (type == "ir") {
        WEB_LOCK();
        String json;
        json.reserve(256 + (resistanceDataCount + resistanceDataCountPairs) * 20);
        json = "{";
        auto addIRData = [&](const char* name, float data[][2], int count) {
            json += "\""; json += name; json += "\":[";
            char buf[40];
            for (int i = 0; i < count; i++) {
                snprintf(buf, sizeof(buf), "[%.3f,%.3f]", data[i][0], data[i][1]);
                json += buf;
                if (i < count - 1) json += ",";
            }
            json += "]";
        };
        addIRData("lu", internalResistanceData, resistanceDataCount); json += ",";
        addIRData("pairs", internalResistanceDataPairs, resistanceDataCountPairs);
        json += "}";
        WEB_UNLOCK();
        server.send(200, "application/json", json);
    }
    else {
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "application/json", "{\"state\":");
        server.sendContent(getJsonState());
        server.sendContent(",\"ambient\":");
        float t[PLOT_WIDTH], h[PLOT_WIDTH], d[PLOT_WIDTH];
        WEB_LOCK();
        memcpy(t, homeScreen.temp_history, sizeof(t));
        memcpy(h, homeScreen.humidity_history, sizeof(h));
        memcpy(d, homeScreen.dew_point_history, sizeof(d));
        WEB_UNLOCK();
        server.sendContent("{");
        streamFloatArrayJSON("t", t, PLOT_WIDTH);
        streamFloatArrayJSON("h", h, PLOT_WIDTH);
        streamFloatArrayJSON("d", d, PLOT_WIDTH, true);
        server.sendContent("}}");
        server.sendContent("");
    }
}

void handleRoot() {
    Serial.println("WEB: handleRoot");
#ifndef MOCK_TEST
    server.sendHeader(F("Connection"), F("close"));
    server.send_P(200, "text/html", INDEX_HTML);
#else
    server.send(200, "text/html", INDEX_HTML);
#endif
}

void handleCommand() {
    String cmd = server.arg("cmd");
    Serial.printf("DEBUG: Web command received: %s\n", cmd.c_str());

    if (cmd == "charge") {
        WEB_LOCK();
        resetAh = true;
        WEB_UNLOCK();
        setBuildModelPhase(BuildModelPhase::Idle);
        setAppState(APP_STATE_BUILDING_MODEL);
    } else if (cmd == "ir") {
        WEB_LOCK();
        currentIRState = IR_STATE_START;
        WEB_UNLOCK();
        setAppState(APP_STATE_MEASURING_IR);
    } else if (cmd == "reset") {
        WEB_LOCK();
        resetAh = true;
        WEB_UNLOCK();
    } else if (cmd == "stop") {
        setAppState(APP_STATE_IDLE);
        applyDuty(0);
    }

    server.send(200, "text/plain", "OK");
}
