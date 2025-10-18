#ifndef DATASTORE_H
#define DATASTORE_H

#include "Shared.h"

class DataStore {
public:
    DataStore();
    void logChargeData(const ChargeLogData& data);
    void updateHistory(double temp1, double temp2, double tempDiff, float voltage, float current);

    // Public data members for now, will be made private with getters later if needed
    float temp1_values[PLOT_WIDTH];
    float temp2_values[PLOT_WIDTH];
    float diff_values[PLOT_WIDTH];
    float voltage_values[PLOT_WIDTH];
    float current_values[PLOT_WIDTH];
    float MAX_DIFF_TEMP;
    uint32_t dutyCycle;
    std::vector<ChargeLogData> chargeLog;
    float internalResistanceData[MAX_RESISTANCE_POINTS][2];
    int resistanceDataCount;
    float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
    int resistanceDataCountPairs;
    float regressedInternalResistanceSlope;
    float regressedInternalResistanceIntercept;
    float regressedInternalResistancePairsSlope;
    float regressedInternalResistancePairsIntercept;

    AppState currentAppState;
    DisplayState currentDisplayState;
};

#endif // DATASTORE_H