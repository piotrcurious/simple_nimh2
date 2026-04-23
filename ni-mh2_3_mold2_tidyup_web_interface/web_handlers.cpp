#include "definitions.h"
#include "home_screen.h"
#include "dashboard_html.h"
#include <WebServer.h>

extern WebServer server;

String getJsonState() {
    String json;
    json.reserve(128);
    char buf[32];
    json = "{";
    json += "\"app\":" + String(currentAppState) + ",";
    json += "\"display\":" + String(currentDisplayState) + ",";
    json += "\"duty\":" + String(dutyCycle) + ",";
    snprintf(buf, sizeof(buf), "\"v\":%.3f,", voltage_mv / 1000.0); json += buf;
    snprintf(buf, sizeof(buf), "\"i\":%.3f,", current_ma / 1000.0); json += buf;
    snprintf(buf, sizeof(buf), "\"mah\":%.3f", (float)mAh_charged); json += buf;
    json += "}";
    return json;
}

static void appendFloatArray(String& json, const char* name, float* arr, int len) {
    json += "\""; json += name; json += "\":[";
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

String getJsonHistory() {
    String json;
    json.reserve(PLOT_WIDTH * 5 * 8); // rough estimate
    json = "{";
    appendFloatArray(json, "t1", temp1_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "t2", temp2_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "td", diff_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "v", voltage_values, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "i", current_values, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonAmbient() {
    String json;
    json.reserve(PLOT_WIDTH * 3 * 8);
    json = "{";
    appendFloatArray(json, "t", homeScreen.temp_history, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "h", homeScreen.humidity_history, PLOT_WIDTH); json += ",";
    appendFloatArray(json, "d", homeScreen.dew_point_history, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonChargeLog() {
    String json;
    json.reserve(chargeLog.size() * 128);
    json = "[";
    char buf[128];
    for (size_t i = 0; i < chargeLog.size(); i++) {
        float td = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        size_t prevIdx = (i > 0) ? i - 1 : 0;
        float localEnergy = 0.0f;
        float estimatedDiff = estimateTempDiff(
                chargeLog[i].voltage, chargeLog[i].voltage, chargeLog[i].current,
                regressedInternalResistancePairsIntercept, chargeLog[i].ambientTemperature,
                chargeLog[i].timestamp, chargeLog[prevIdx].timestamp, chargeLog[i].batteryTemperature,
                &localEnergy
            );
        float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;

        snprintf(buf, sizeof(buf),
            "{\"t\":%lu,\"i\":%.3f,\"v\":%.3f,\"at\":%.2f,\"bt\":%.2f,\"d\":%d,\"irlu\":%.3f,\"irp\":%.3f,\"td\":%.2f,\"th\":%.2f}",
            chargeLog[i].timestamp, chargeLog[i].current, chargeLog[i].voltage,
            chargeLog[i].ambientTemperature, chargeLog[i].batteryTemperature,
            chargeLog[i].dutyCycle, chargeLog[i].internalResistanceLoadedUnloaded,
            chargeLog[i].internalResistancePairs, td, thresholdValue);

        json += buf;
        if (i < chargeLog.size() - 1) json += ",";
    }
    json += "]";
    return json;
}

String getJsonIR() {
    String json;
    json.reserve(256 + (resistanceDataCount + resistanceDataCountPairs) * 20);
    json = "{";
    auto addIRData = [&](const char* name, float data[][2], int count) {
        json += "\""; json += name; json += "\":[";
        char buf[32];
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
    return json;
}

void handleData() {
    String type = server.arg("type");
    if (type == "state") server.send(200, "application/json", getJsonState());
    else if (type == "history") server.send(200, "application/json", getJsonHistory());
    else if (type == "ambient") server.send(200, "application/json", getJsonAmbient());
    else if (type == "chargelog") server.send(200, "application/json", getJsonChargeLog());
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
    if (cmd == "charge") {
        resetAh = true;
        currentAppState = APP_STATE_BUILDING_MODEL;
    } else if (cmd == "ir") {
        currentAppState = APP_STATE_MEASURING_IR;
        extern IRState currentIRState;
        currentIRState = IR_STATE_START;
    } else if (cmd == "reset") {
        resetAh = true;
    } else if (cmd == "stop") {
        currentAppState = APP_STATE_IDLE;
        applyDuty(0);
    }
    server.send(200, "text/plain", "OK");
}
