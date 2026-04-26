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

/**
 * HELPER: Streams a float array as JSON directly to the client.
 * This prevents creating a giant String in the heap.
 */
static void streamFloatArrayJSON(const char* name, float* arr, int len, bool isLast = false) {
    server.sendContent("\"");
    server.sendContent(name);
    server.sendContent("\":[");
    
    char buf[16];
    for (int i = 0; i < len; i++) {
        if (std::isnan(arr[i])) {
            server.sendContent("null");
        } else {
            snprintf(buf, sizeof(buf), "%.2f", arr[i]);
            server.sendContent(buf);
        }
        if (i < len - 1) server.sendContent(",");
    }
    
    server.sendContent(isLast ? "]" : "],");
}

/**
 * REPLACEMENT: getJsonState()
 * Returns a small string (safe, as it's < 200 bytes)
 */
String getJsonState() {
    char buf[256];
    extern float noiseFloorMv;
    snprintf(buf, sizeof(buf),
        "{\"app\":%d,\"display\":%d,\"duty\":%d,\"v\":%.3f,\"i\":%.3f,\"mah\":%.3f,\"max_dt\":%.2f,\"phase\":%d,\"offset\":%.2f,\"noise\":%.2f}",
        currentAppState, currentDisplayState, dutyCycle, 
        voltage_mv / 1000.0f, current_ma / 1000.0f, (float)mAh_charged,
        MAX_DIFF_TEMP, (int)buildModelPhase, systemData.getCurrentZeroOffsetMv(), noiseFloorMv);
    return String(buf);
}

/**
 * CBOR WRITER CLASS
 */
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
            uint8_t tmp[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
            putBytes(tmp, 2);
        } else if (val <= 0xFFFFFFFFULL) {
            put((major << 5) | 26);
            uint8_t tmp[4] = { (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
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

/**
 * Unified Binary Sender
 * FIXED: Uses startAtomicResponse/sendContent pattern for raw binary buffers
 */
static void sendBinaryResponse(const char* contentType, const std::vector<uint8_t>& payload) {
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.sendHeader(F("Connection"), F("close"));
    // We send an empty string content but specify the exact length to trigger content output
    server.setContentLength(payload.size());
    server.send(200, contentType, ""); 
    if (!payload.empty()) {
        server.sendContent((const char*)payload.data(), payload.size());
    }
}

/**
 * STREAMING JSON HANDLERS
 */
void streamJsonHistory() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "{");
    streamFloatArrayJSON("t1", temp1_values, PLOT_WIDTH);
    streamFloatArrayJSON("t2", temp2_values, PLOT_WIDTH);
    streamFloatArrayJSON("td", diff_values, PLOT_WIDTH);
    streamFloatArrayJSON("v", voltage_values, PLOT_WIDTH);
    streamFloatArrayJSON("i", current_values, PLOT_WIDTH, true);
    server.sendContent("}");
    server.sendContent(""); 
}

void streamJsonAmbient() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "{");
    streamFloatArrayJSON("t", homeScreen.temp_history, PLOT_WIDTH);
    streamFloatArrayJSON("h", homeScreen.humidity_history, PLOT_WIDTH);
    streamFloatArrayJSON("d", homeScreen.dew_point_history, PLOT_WIDTH, true);
    server.sendContent("}");
    server.sendContent("");
}

void streamJsonChargeLog() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "[");
    char buf[200];
    for (size_t i = 0; i < chargeLog.size(); i++) {
        float td = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        float localEnergy = 0.0f;
        float estimatedDiff = estimateTempDiff(
            chargeLog[i].voltage, chargeLog[i].voltage, chargeLog[i].current,
            regressedInternalResistancePairsIntercept, chargeLog[i].ambientTemperature,
            chargeLog[i].timestamp, (i > 0 ? chargeLog[i-1].timestamp : chargeLog[i].timestamp), 
            chargeLog[i].batteryTemperature, &localEnergy
        );
        float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;

        snprintf(buf, sizeof(buf),
            "{\"t\":%u,\"i\":%.3f,\"v\":%.3f,\"at\":%.2f,\"bt\":%.2f,\"d\":%d,\"irlu\":%.3f,\"irp\":%.3f,\"td\":%.2f,\"th\":%.2f}",
            (unsigned int)chargeLog[i].timestamp, chargeLog[i].current, chargeLog[i].voltage,
            chargeLog[i].ambientTemperature, chargeLog[i].batteryTemperature, chargeLog[i].dutyCycle,
            chargeLog[i].internalResistanceLoadedUnloaded, chargeLog[i].internalResistancePairs,
            td, thresholdValue
        );

        server.sendContent(buf);
        if (i < chargeLog.size() - 1) server.sendContent(",");
    }
    server.sendContent("]");
    server.sendContent("");
}

/**
 * CBOR Data Builders
 */
static void appendCborState(CborWriter& w) {
    extern float noiseFloorMv;
    w.startMap(10);
    w.addText("app");    w.addInt(currentAppState);
    w.addText("display");w.addInt(currentDisplayState);
    w.addText("duty");   w.addInt(dutyCycle);
    w.addText("v");      w.addFloat(voltage_mv / 1000.0f);
    w.addText("i");      w.addFloat(current_ma / 1000.0f);
    w.addText("mah");    w.addFloat((float)mAh_charged);
    w.addText("max_dt"); w.addFloat(MAX_DIFF_TEMP);
    w.addText("phase");  w.addInt((int)buildModelPhase);
    w.addText("offset"); w.addFloat(systemData.getCurrentZeroOffsetMv());
    w.addText("noise");  w.addFloat(noiseFloorMv);
}

static void appendCborChargeLog(CborWriter& w) {
    w.startArray(chargeLog.size());
    for (size_t i = 0; i < chargeLog.size(); i++) {
        float td = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        float localEnergy = 0.0f;
        float estimatedDiff = estimateTempDiff(
            chargeLog[i].voltage, chargeLog[i].voltage, chargeLog[i].current,
            regressedInternalResistancePairsIntercept, chargeLog[i].ambientTemperature,
            chargeLog[i].timestamp, (i > 0 ? chargeLog[i-1].timestamp : chargeLog[i].timestamp), 
            chargeLog[i].batteryTemperature, &localEnergy
        );
        w.startMap(10);
        w.addText("t");    w.addUInt(chargeLog[i].timestamp);
        w.addText("i");    w.addFloat(chargeLog[i].current);
        w.addText("v");    w.addFloat(chargeLog[i].voltage);
        w.addText("at");   w.addFloat(chargeLog[i].ambientTemperature);
        w.addText("bt");   w.addFloat(chargeLog[i].batteryTemperature);
        w.addText("d");    w.addInt(chargeLog[i].dutyCycle);
        w.addText("irlu"); w.addFloat(chargeLog[i].internalResistanceLoadedUnloaded);
        w.addText("irp");  w.addFloat(chargeLog[i].internalResistancePairs);
        w.addText("td");   w.addFloat(td);
        w.addText("th");   w.addFloat(MAX_TEMP_DIFF_THRESHOLD + estimatedDiff);
    }
}

/**
 * WEB HANDLERS
 */
void handleData() {
    String type = server.arg("type");
    bool wantCbor = (server.arg("fmt") == "cbor");

    if (wantCbor) {
        CborWriter w;
        if (type == "state") {
            w.reserve(256);
            appendCborState(w);
            sendBinaryResponse("application/cbor", w.data);
        } 
        else if (type == "history") {
            w.reserve(PLOT_WIDTH * 5 * 4 + 100);
            w.startMap(5);
            w.addText("t1"); cborAddFloatArray(w, temp1_values, PLOT_WIDTH);
            w.addText("t2"); cborAddFloatArray(w, temp2_values, PLOT_WIDTH);
            w.addText("td"); cborAddFloatArray(w, diff_values, PLOT_WIDTH);
            w.addText("v");  cborAddFloatArray(w, voltage_values, PLOT_WIDTH);
            w.addText("i");  cborAddFloatArray(w, current_values, PLOT_WIDTH);
            sendBinaryResponse("application/cbor", w.data);
        }
        else if (type == "chargelog") {
            w.reserve(chargeLog.size() * 80 + 100);
            appendCborChargeLog(w);
            sendBinaryResponse("application/cbor", w.data);
        }
        else if (type == "ir") {
            w.reserve(512 + (resistanceDataCount + resistanceDataCountPairs) * 16);
            w.startMap(2);
            w.addText("lu");    cborAddXYPairs(w, internalResistanceData, resistanceDataCount);
            w.addText("pairs"); cborAddXYPairs(w, internalResistanceDataPairs, resistanceDataCountPairs);
            sendBinaryResponse("application/cbor", w.data);
        }
        else {
            w.reserve(1024);
            w.startMap(2);
            w.addText("state");   appendCborState(w);
            w.addText("ambient"); 
            w.startMap(3);
            w.addText("t"); cborAddFloatArray(w, homeScreen.temp_history, PLOT_WIDTH);
            w.addText("h"); cborAddFloatArray(w, homeScreen.humidity_history, PLOT_WIDTH);
            w.addText("d"); cborAddFloatArray(w, homeScreen.dew_point_history, PLOT_WIDTH);
            sendBinaryResponse("application/cbor", w.data);
        }
        return;
    }

    if (type == "state") {
        server.send(200, "application/json", getJsonState());
    } else if (type == "history") {
        streamJsonHistory();
    } else if (type == "ambient") {
        streamJsonAmbient();
    } else if (type == "chargelog") {
        streamJsonChargeLog();
    } else {
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "application/json", "{\"state\":");
        server.sendContent(getJsonState());
        server.sendContent(",\"ambient\":");
        // We can't easily stream nested objects from multiple functions, 
        // but for small enough objects we can build them:
        String amb = "{";
        amb += "\"t\":[],\"h\":[],\"d\":[]}"; // Placeholder or logic to stream specifically
        server.sendContent(amb);
        server.sendContent("}"); 
        server.sendContent("");
    }
}

void handleRoot() {
    server.sendHeader(F("Connection"), F("close"));
    server.send(200, "text/html", INDEX_HTML);
}

void handleCommand() {
    String cmd = server.arg("cmd");
    if (cmd == "charge") {
        resetAh = true;
        buildModelPhase = BuildModelPhase::Idle;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        currentAppState = APP_STATE_BUILDING_MODEL;
    } else if (cmd == "stop") {
        currentAppState = APP_STATE_IDLE;
        applyDuty(0);
    }
    server.send(200, "text/plain", "OK");
}
