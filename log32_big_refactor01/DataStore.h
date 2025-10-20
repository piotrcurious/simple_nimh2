#ifndef DATASTORE_H
#define DATASTORE_H

#include "Shared.h"

class DataStore {
public:
    // State Management
    AppState app_state;

    // Real-time Sensor Data
    volatile float voltage_mv;
    volatile float current_ma;
    volatile float mAh_charged;

    // Current Estimation Model
    CurrentModel current_model;

    // Flags and Timers
    volatile bool resetAh;
    volatile uint32_t mAh_last_time;
    uint32_t dutyCycle;
    bool isCharging;
    bool isMeasuringResistance;
    ChargingState chargingState;
    int cachedOptimalDuty;
    unsigned long lastPlotUpdateTime;
    unsigned long lastChargingHouseTime;
    unsigned long lastIRHandleTime;
    uint8_t overtemp_trip_counter;
    unsigned long chargePhaseStartTime;
    unsigned long chargingStartTime;
    unsigned long lastChargeEvaluationTime;

    // Data Arrays and History
    float temp1_values[320]; // PLOT_WIDTH
    float temp2_values[320];
    float diff_values[320];
    float voltage_values[320];
    float current_values[320];
    float internalResistanceData[100][2]; // MAX_RESISTANCE_POINTS
    int resistanceDataCount;
    float internalResistanceDataPairs[100][2];
    int resistanceDataCountPairs;
    float regressedInternalResistanceSlope;
    float regressedInternalResistanceIntercept;
    float regressedInternalResistancePairsSlope;
    float regressedInternalResistancePairsIntercept;
    std::vector<ChargeLogData> chargeLog;


    DataStore() :
        app_state(APP_STATE_IDLE),
        voltage_mv(1000.0f),
        current_ma(0.0f),
        mAh_charged(0.0f),
        resetAh(false),
        mAh_last_time(0),
        dutyCycle(0),
        isCharging(false),
        isMeasuringResistance(false),
        chargingState(CHARGE_IDLE),
        cachedOptimalDuty(0),
        resistanceDataCount(0),
        resistanceDataCountPairs(0),
        regressedInternalResistanceSlope(0.0f),
        regressedInternalResistanceIntercept(0.0f),
        regressedInternalResistancePairsSlope(0.0f),
        regressedInternalResistancePairsIntercept(0.0f),
        lastPlotUpdateTime(0),
        lastChargingHouseTime(0),
        lastIRHandleTime(0),
        overtemp_trip_counter(0),
        chargePhaseStartTime(0),
        chargingStartTime(0),
        lastChargeEvaluationTime(0)
    {
        // Initialize arrays
        for (int i = 0; i < 320; ++i) {
            temp1_values[i] = 25.0f;
            temp2_values[i] = 25.0f;
            diff_values[i] = 0.0f;
            voltage_values[i] = 1.0f;
            current_values[i] = 0.0f;
        }
    }
};

#endif // DATASTORE_H