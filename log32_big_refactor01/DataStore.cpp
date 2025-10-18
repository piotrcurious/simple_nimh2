#include "DataStore.h"

DataStore::DataStore() :
    MAX_DIFF_TEMP(1.5),
    dutyCycle(0),
    resistanceDataCount(0),
    resistanceDataCountPairs(0),
    regressedInternalResistanceSlope(0),
    regressedInternalResistanceIntercept(0),
    regressedInternalResistancePairsSlope(0),
    regressedInternalResistancePairsIntercept(0),
    currentAppState(APP_STATE_IDLE),
    currentDisplayState(DISPLAY_STATE_MAIN)
{
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f;
        current_values[i] = 0.0f;
    }
}

void DataStore::logChargeData(const ChargeLogData& data) {
    chargeLog.push_back(data);
}

void DataStore::updateHistory(double temp1, double temp2, double tempDiff, float voltage, float current) {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        temp2_values[i] = temp2_values[i + 1];
        diff_values[i] = diff_values[i + 1];
        voltage_values[i] = voltage_values[i + 1];
        current_values[i] = current_values[i + 1];
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    temp2_values[PLOT_WIDTH - 1] = temp2;
    diff_values[PLOT_WIDTH - 1] = tempDiff;
    voltage_values[PLOT_WIDTH - 1] = voltage;
    current_values[PLOT_WIDTH - 1] = current;
}