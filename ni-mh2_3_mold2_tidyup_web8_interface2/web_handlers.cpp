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

#ifndef MOCK_TEST
#define WEB_LOCK() if (webDataMutex) xSemaphoreTake(webDataMutex, portMAX_DELAY)
#define WEB_UNLOCK() if (webDataMutex) xSemaphoreGive(webDataMutex)
#else
#define WEB_LOCK()
#define WEB_UNLOCK()
#endif

static void appendFloatArray(String& json, const char* name, float* arr, int len) {
    json += "\"";
    json += name;
    json += "\":[";
    char buf[16];
    for (int i = 0; i < len; i++) {
        if (std::isnan(arr[i])) json += "null";
        else {
            snprintf(buf, sizeof(buf), "%.2f", arr[i]);
            json += buf;
        }
        if (i < len - 1) json += ",";
    }
    json += "]";
}

String getJsonState() {
    WEB_LOCK();
    String json;
    json.reserve(160);
    char buf[32];
    json = "{";
    json += "\"app\":" + String(currentAppState) + ",";
    json += "\"display\":" + String(currentDisplayState) + ",";
    json += "\"duty\":" + String(dutyCycle) + ",";
    snprintf(buf, sizeof(buf), "\"v\":%.3f,", voltage_mv / 1000.0f); json += buf;
    snprintf(buf, sizeof(buf), "\"i\":%.3f,", current_ma / 1000.0f); json += buf;
    snprintf(buf, sizeof(buf), "\"mah\":%.3f,", (float)mAh_charged); json += buf;
    snprintf(buf, sizeof(buf), "\"max_dt\":%.2f,", MAX_DIFF_TEMP); json += buf;
    json += "\"phase\":" + String((int)buildModelPhase) + ",";
    snprintf(buf, sizeof(buf), "\"offset\":%.2f,", systemData.getCurrentZeroOffsetMv()); json += buf;
    extern float noiseFloorMv;
    snprintf(buf, sizeof(buf), "\"noise\":%.2f", noiseFloorMv); json += buf;
    json += "}";
    WEB_UNLOCK();
    return json;
}

String getJsonHistory() {
    WEB_LOCK();
    String json;
    json.reserve(PLOT_WIDTH * 5 * 8);
    json = "{";
    appendFloatArray(json, "t1", temp1_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "t2", temp2_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "td", diff_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "v", voltage_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "i", current_values, PLOT_WIDTH);
    json += "}";
    WEB_UNLOCK();
    return json;
}

String getJsonAmbient() {
    WEB_LOCK();
    String json;
    json.reserve(PLOT_WIDTH * 3 * 8);
    json = "{";
    appendFloatArray(json, "t", homeScreen.temp_history, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "h", homeScreen.humidity_history, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "d", homeScreen.dew_point_history, PLOT_WIDTH);
    json += "}";
    WEB_UNLOCK();
    return json;
}


String getJsonIR() {
    WEB_LOCK();
    String json;
    json.reserve(256 + (resistanceDataCount + resistanceDataCountPairs) * 20);
    json = "{";
    auto addIRData = [&](const char* name, float data[][2], int count) {
        json += "\"";
        json += name;
        json += "\":[";
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
    return json;
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
        if (val < 24) {
            put((major << 5) | (uint8_t)val);
        } else if (val <= 0xFF) {
            put((major << 5) | 24);
            put((uint8_t)val);
        } else if (val <= 0xFFFF) {
            put((major << 5) | 25);
            uint16_t v = (uint16_t)val;
            uint8_t tmp[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
            putBytes(tmp, 2);
        } else if (val <= 0xFFFFFFFFULL) {
            put((major << 5) | 26);
            uint32_t v = (uint32_t)val;
            uint8_t tmp[4] = {
                (uint8_t)(v >> 24), (uint8_t)(v >> 16),
                (uint8_t)(v >> 8), (uint8_t)(v & 0xFF)
            };
            putBytes(tmp, 4);
        } else {
            put((major << 5) | 27);
            uint8_t tmp[8] = {
                (uint8_t)(val >> 56), (uint8_t)(val >> 48), (uint8_t)(val >> 40), (uint8_t)(val >> 32),
                (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)
            };
            putBytes(tmp, 8);
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
        uint8_t tmp[4] = {
            (uint8_t)(u >> 24), (uint8_t)(u >> 16), (uint8_t)(u >> 8), (uint8_t)(u & 0xFF)
        };
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
    extern float noiseFloorMv;
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
    w.addText("noise");  w.addFloat(noiseFloorMv);
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
    NetworkClient &client = server.client();

    client.print(F("HTTP/1.1 200 OK\r\n"));
    client.print(F("Cache-Control: no-store\r\n"));
    client.print(F("Connection: close\r\n"));
    client.print(F("Content-Type: "));
    client.print(contentType);
    client.print(F("\r\nContent-Length: "));
    client.print((unsigned long)payload.size());
    client.print(F("\r\n\r\n"));

    if (!payload.empty()) {
        client.write(payload.data(), payload.size());
    }
    client.flush();
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
    {
        WEB_LOCK();
        total = chargeLog.size();
        WEB_UNLOCK();
    }

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/cbor", "");

    float currentRParam = 0.0f;
    {
        WEB_LOCK();
        currentRParam = regressedInternalResistancePairsIntercept;
        WEB_UNLOCK();
    }

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

        // If it's the very first batch, we need to start the array.
        // CBOR arrays can be indefinite length.
        if (i == 0) {
            w.put(0x9F); // Start indefinite length array
        }

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

        server.sendContent((const char*)w.data.data(), w.data.size());
    }

    uint8_t stopByte = 0xFF; // Break for indefinite length array
    server.sendContent((const char*)&stopByte, 1);
    server.sendContent(""); // Finish chunked encoding
#endif
}

static void sendCborRoot() {
    WEB_LOCK();
    CborWriter w;
    w.reserve(512);
    w.startMap(2);
    w.addText("state");
    appendCborState(w);
    w.addText("ambient");
    appendCborAmbient(w);
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

    char buf[256];
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
    String type = server.arg("type");
    bool wantCbor = server.arg("fmt") == "cbor";

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
    else if (type == "history") server.send(200, "application/json", getJsonHistory());
    else if (type == "ambient") server.send(200, "application/json", getJsonAmbient());
    else if (type == "chargelog") streamJsonChargeLog();
    else if (type == "ir") server.send(200, "application/json", getJsonIR());
    else {
        String json = "{";
        json += "\"state\":" + getJsonState() + ",";
        json += "\"ambient\":" + getJsonAmbient();
        json += "}";
        server.send(200, "application/json", json);
    }
}

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleCommand() {
    String cmd = server.arg("cmd");
    Serial.printf("DEBUG: Web command received: %s\n", cmd.c_str());

    if (cmd == "charge") {
        resetAh = true;
        buildModelPhase = BuildModelPhase::Idle;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        currentAppState = APP_STATE_BUILDING_MODEL;
    } else if (cmd == "ir") {
        currentIRState = IR_STATE_START;
        currentAppState = APP_STATE_MEASURING_IR;
    } else if (cmd == "reset") {
        resetAh = true;
    } else if (cmd == "stop") {
        currentAppState = APP_STATE_IDLE;
        applyDuty(0);
    }

    server.send(200, "text/plain", "OK");
}
