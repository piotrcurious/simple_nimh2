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

#ifndef MOCK_TEST
extern AsyncWebServer server;
extern AsyncWebSocket ws;
#else
extern AsyncWebSocket ws;
#endif
extern void setAppState(AppState s);
extern void setBuildModelPhase(BuildModelPhase p);

static void addFloatArrayToJson(String& json, const char* name, const float* arr, int len, bool isLast = false) {
    json += "\"";
    json += name;
    json += "\":[";
    for (int i = 0; i < len; i++) {
        if (std::isnan(arr[i])) json += "null";
        else {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f", arr[i]);
            json += buf;
        }
        if (i < len - 1) json += ",";
    }
    json += isLast ? "]" : "],";
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

static String getJsonHistory() {
    float t1[PLOT_WIDTH], t2[PLOT_WIDTH], td[PLOT_WIDTH], v[PLOT_WIDTH], i[PLOT_WIDTH];
    WEB_LOCK();
    memcpy(t1, temp1_values, sizeof(t1));
    memcpy(t2, temp2_values, sizeof(t2));
    memcpy(td, diff_values, sizeof(td));
    memcpy(v, voltage_values, sizeof(v));
    memcpy(i, current_values, sizeof(i));
    WEB_UNLOCK();

    String json = "{";
    addFloatArrayToJson(json, "t1", t1, PLOT_WIDTH);
    addFloatArrayToJson(json, "t2", t2, PLOT_WIDTH);
    addFloatArrayToJson(json, "td", td, PLOT_WIDTH);
    addFloatArrayToJson(json, "v", v, PLOT_WIDTH);
    addFloatArrayToJson(json, "i", i, PLOT_WIDTH, true);
    json += "}";
    return json;
}

static String getJsonAmbient() {
    float t[PLOT_WIDTH], h[PLOT_WIDTH], d[PLOT_WIDTH];
    WEB_LOCK();
    memcpy(t, homeScreen.temp_history, sizeof(t));
    memcpy(h, homeScreen.humidity_history, sizeof(h));
    memcpy(d, homeScreen.dew_point_history, sizeof(d));
    WEB_UNLOCK();

    String json = "{";
    addFloatArrayToJson(json, "t", t, PLOT_WIDTH);
    addFloatArrayToJson(json, "h", h, PLOT_WIDTH);
    addFloatArrayToJson(json, "d", d, PLOT_WIDTH, true);
    json += "}";
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

static void sendCborState(AsyncWebSocketClient *client) {
    if (!client) return;
    WEB_LOCK();
    CborWriter w;
    w.reserve(256);
    appendCborState(w);
    WEB_UNLOCK();
    client->binary(w.data.data(), w.data.size());
}

static void sendCborHistory(AsyncWebSocketClient *client) {
    if (!client) return;
    WEB_LOCK();
    CborWriter w;
    w.reserve(PLOT_WIDTH * 5 * 8 + 32);
    appendCborHistory(w);
    WEB_UNLOCK();
    client->binary(w.data.data(), w.data.size());
}

static void sendCborAmbient(AsyncWebSocketClient *client) {
    if (!client) return;
    WEB_LOCK();
    CborWriter w;
    w.reserve(PLOT_WIDTH * 3 * 8 + 32);
    appendCborAmbient(w);
    WEB_UNLOCK();
    client->binary(w.data.data(), w.data.size());
}

static void sendCborIR(AsyncWebSocketClient *client) {
    if (!client) return;
    WEB_LOCK();
    CborWriter w;
    w.reserve(256 + (resistanceDataCount + resistanceDataCountPairs) * 24);
    appendCborIR(w);
    WEB_UNLOCK();
    client->binary(w.data.data(), w.data.size());
}

/*
static void sendCborChargeLog() {
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

    // Send the array header (definite length)
    {
        CborWriter w;
        w.addTypeVal(4, total);
        server.client().write(w.data.data(), w.data.size());
    }

    for (size_t i = 0; i < total; i += batchSize) {
        vTaskDelay(0); // Yield to other tasks of same priority
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

    server.sendContent("");
}
*/

static void sendCborChargeLog(AsyncWebSocketClient *client) {
    if (!client) return;
    size_t total = 0;
    float currentRParam = 0.0f;
    WEB_LOCK();
    total = chargeLog.size();
    currentRParam = regressedInternalResistancePairsIntercept;
    WEB_UNLOCK();

    const size_t batchSize = 10;
    uint32_t lastTimestamp = 0;

    for (size_t i = 0; i < total; i += batchSize) {
        vTaskDelay(0);
        CborWriter w;
        w.reserve(batchSize * 100 + 32);

        // Individual packets must be valid CBOR objects.
        // We wrap each batch in a map with "log" and "count" for the frontend to process incrementally.
        size_t itemsInBatch = std::min(batchSize, total - i);
        w.startMap(2);
        w.addText("batch"); w.startArray(itemsInBatch);

        for (size_t j = 0; j < itemsInBatch; j++) {
            ChargeLogData entry;
            WEB_LOCK();
            if ((i + j) < chargeLog.size()) entry = chargeLog[i + j];
            WEB_UNLOCK();

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
        w.addText("total"); w.addUInt(total);
        client->binary(w.data.data(), w.data.size());
    }
}

static void sendCborRoot(AsyncWebSocketClient *client) {
    if (!client) return;
    WEB_LOCK();
    CborWriter w;
    w.reserve(512);
    w.startMap(2);
    w.addText("state");   appendCborState(w);
    w.addText("ambient"); appendCborAmbient(w);
    WEB_UNLOCK();
    client->binary(w.data.data(), w.data.size());
}

static void handleJsonChargeLog(AsyncWebServerRequest *request) {
    size_t total = 0;
    float currentRParam = 0.0f;
    WEB_LOCK();
    total = chargeLog.size();
    currentRParam = regressedInternalResistancePairsIntercept;
    WEB_UNLOCK();

    AsyncWebServerResponse *response = request->beginChunkedResponse("application/json",
        [total, currentRParam](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            static size_t currentEntry = 0;
            static uint32_t lastTimestamp = 0;
            if (index == 0) {
                currentEntry = 0;
                lastTimestamp = 0;
            }
            if (currentEntry >= total) return 0;

            String s = "";
            if (index == 0) s += "[";

            while (currentEntry < total) {
                ChargeLogData entry;
                WEB_LOCK();
                if (currentEntry < chargeLog.size()) entry = chargeLog[currentEntry];
                WEB_UNLOCK();

                float td = entry.batteryTemperature - entry.ambientTemperature;
                float localEnergy = 0.0f;
                uint32_t prevTs = (currentEntry == 0) ? entry.timestamp : lastTimestamp;
                float estimatedDiff = estimateTempDiff(
                    entry.voltage, entry.voltage, entry.current,
                    currentRParam, entry.ambientTemperature,
                    entry.timestamp, prevTs, entry.batteryTemperature,
                    &localEnergy
                );
                float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;

                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"t\":%u,\"i\":%.3f,\"v\":%.3f,\"at\":%.2f,\"bt\":%.2f,\"d\":%d,\"irlu\":%.3f,\"irp\":%.3f,\"td\":%.2f,\"th\":%.2f}",
                    (unsigned int)entry.timestamp, entry.current, entry.voltage, entry.ambientTemperature,
                    entry.batteryTemperature, entry.dutyCycle, entry.internalResistanceLoadedUnloaded,
                    entry.internalResistancePairs, td, thresholdValue
                );

                String item = buf;
                if (currentEntry < total - 1) item += ",";
                else item += "]";

                if (s.length() + item.length() > maxLen) break;

                s += item;
                lastTimestamp = entry.timestamp;
                currentEntry++;
            }

            memcpy(buffer, s.c_str(), s.length());
            return s.length();
        });

    response->addHeader("Cache-Control", "no-store");
    request->send(response);
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

static void processCommand(String cmd, AsyncWebSocketClient *client = nullptr) {
    if (cmd == "REQ_STATE") { sendCborState(client); return; }
    if (cmd == "REQ_HISTORY") { sendCborHistory(client); return; }
    if (cmd == "REQ_AMBIENT") { sendCborAmbient(client); return; }
    if (cmd == "REQ_CHARGELOG") { sendCborChargeLog(client); return; }
    if (cmd == "REQ_IR") { sendCborIR(client); return; }

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
}

void handleCommand(AsyncWebServerRequest *request) {
    String cmd = request->arg("cmd");
    Serial.printf("DEBUG: Web command received: %s\n", cmd.c_str());
    processCommand(cmd);
    request->send(200, "text/plain", "OK");
}

#if !defined(MOCK_TEST) || defined(MOCK_TEST)
void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS Client connected [%u]\n", client->id());
        sendCborState(client);
        sendCborAmbient(client);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS Client disconnected [%u]\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len) {
            if (info->opcode == WS_TEXT) {
                char* buf = (char*)malloc(len + 1);
                memcpy(buf, data, len);
                buf[len] = 0;
                String cmd = buf;
                free(buf);
                Serial.printf("WS Command received: %s\n", cmd.c_str());
                processCommand(cmd, client);
            }
        }
    }
}

void broadcastLiveTelemetry() {
    static uint32_t lastBroadcast = 0;
    if (millis() - lastBroadcast < 1000) return;
    lastBroadcast = millis();

    if (ws.count() == 0) return;

    WEB_LOCK();
    CborWriter w;
    w.reserve(256);
    w.startMap(1);
    w.addText("state");
    appendCborState(w);
    WEB_UNLOCK();

    ws.binaryAll(w.data.data(), w.data.size());
}
#endif
