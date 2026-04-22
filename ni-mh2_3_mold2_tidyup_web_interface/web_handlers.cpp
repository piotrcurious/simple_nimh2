#include "definitions.h"
#include "home_screen.h"
#include "dashboard_html.h"
#include <WebServer.h>

extern WebServer server;

String getJsonState() {
    String json = "{";
    json += "\"app\":" + String(currentAppState) + ",";
    json += "\"display\":" + String(currentDisplayState) + ",";
    json += "\"duty\":" + String(dutyCycle) + ",";
    json += "\"v\":" + String(voltage_mv / 1000.0, 3) + ",";
    json += "\"i\":" + String(current_ma / 1000.0, 3) + ",";
    json += "\"mah\":" + String(mAh_charged, 3);
    json += "}";
    return json;
}

String getJsonHistory() {
    String json = "{";
    auto addArray = [&](const String& name, float* arr, int len) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < len; i++) {
            if (std::isnan(arr[i])) json += "null";
            else json += String(arr[i], 2);
            if (i < len - 1) json += ",";
        }
        json += "]";
    };
    addArray("t1", temp1_values, PLOT_WIDTH); json += ",";
    addArray("t2", temp2_values, PLOT_WIDTH); json += ",";
    addArray("td", diff_values, PLOT_WIDTH); json += ",";
    addArray("v", voltage_values, PLOT_WIDTH); json += ",";
    addArray("i", current_values, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonAmbient() {
    String json = "{";
    auto addArray = [&](const String& name, float* arr, int len) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < len; i++) {
            if (std::isnan(arr[i])) json += "null";
            else json += String(arr[i], 2);
            if (i < len - 1) json += ",";
        }
        json += "]";
    };
    addArray("t", homeScreen.temp_history, PLOT_WIDTH); json += ",";
    addArray("h", homeScreen.humidity_history, PLOT_WIDTH); json += ",";
    addArray("d", homeScreen.dew_point_history, PLOT_WIDTH);
    json += "}";
    return json;
}

String getJsonChargeLog() {
    String json = "[";
    for (size_t i = 0; i < chargeLog.size(); i++) {
        json += "{";
        json += "\"t\":" + String(chargeLog[i].timestamp) + ",";
        json += "\"i\":" + String(chargeLog[i].current, 3) + ",";
        json += "\"v\":" + String(chargeLog[i].voltage, 3) + ",";
        json += "\"at\":" + String(chargeLog[i].ambientTemperature, 2) + ",";
        json += "\"bt\":" + String(chargeLog[i].batteryTemperature, 2) + ",";
        json += "\"d\":" + String(chargeLog[i].dutyCycle) + ",";
        json += "\"irlu\":" + String(chargeLog[i].internalResistanceLoadedUnloaded, 3) + ",";
        json += "\"irp\":" + String(chargeLog[i].internalResistancePairs, 3) + ",";

        float td = chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature;
        json += "\"td\":" + String(td, 2) + ",";

        // Calculate estimated threshold like in graphing.cpp
        size_t prevIdx = (i > 0) ? i - 1 : 0;
        float localEnergy = 0.0f;
        float estimatedDiff = estimateTempDiff(
                chargeLog[i].voltage,
                chargeLog[i].voltage,
                chargeLog[i].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[i].ambientTemperature,
                chargeLog[i].timestamp,
                chargeLog[prevIdx].timestamp,
                chargeLog[i].batteryTemperature,
                &localEnergy,
                DEFAULT_CELL_MASS_KG,
                DEFAULT_SPECIFIC_HEAT,
                DEFAULT_SURFACE_AREA_M2,
                DEFAULT_CONVECTIVE_H,
                DEFAULT_EMISSIVITY
            );
        float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
        json += "\"th\":" + String(thresholdValue, 2);

        json += "}";
        if (i < chargeLog.size() - 1) json += ",";
    }
    json += "]";
    return json;
}

String getJsonIR() {
    String json = "{";
    auto addIRData = [&](const String& name, float data[][2], int count) {
        json += "\"" + name + "\":[";
        for (int i = 0; i < count; i++) {
            json += "[" + String(data[i][0], 3) + "," + String(data[i][1], 3) + "]";
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
