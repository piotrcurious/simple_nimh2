#include "charging.h"
#include "definitions.h"
#include "logging.h"

extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
extern float estimateCurrent(int dutyCycle);
extern AppState currentAppState;

#ifndef MOCK_TEST
unsigned long chargingStartTime = 0;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;
float currentRampTarget = 0.0f;

// Monitoring evaluation snapshots (persist across states)
float eval_mAh_snapshot = 0.0f;       // mAh at start of monitor evaluation interval
unsigned long eval_time_snapshot = 0; // ms at start of monitor evaluation interval

// Re-evaluation (binary-search) tracking for logging (does NOT affect monitor snapshots)
bool reeval_active = false;
float reeval_start_mAh = 0.0f;
unsigned long reeval_start_ms = 0;

// last re-eval metrics for logging/inspection (optional)
float lastReeval_delta_mAh = 0.0f;
unsigned long lastReeval_duration_ms = 0;
float lastReeval_avgCurrent_A = 0.0f;

AsyncMeasure meas;
FindOptManager findOpt;
RemeasureManager remeasure;
#else
extern unsigned long chargingStartTime;
extern ChargingState chargingState;
extern int cachedOptimalDuty;
extern unsigned long chargePhaseStartTime;
extern uint8_t overtemp_trip_counter;
extern unsigned long lastChargeEvaluationTime;
extern float maximumCurrent;
extern float currentRampTarget;
extern float eval_mAh_snapshot;
extern unsigned long eval_time_snapshot;
extern bool reeval_active;
extern float reeval_start_mAh;
extern unsigned long reeval_start_ms;
extern float lastReeval_delta_mAh;
extern unsigned long lastReeval_duration_ms;
extern float lastReeval_avgCurrent_A;
extern AsyncMeasure meas;
extern FindOptManager findOpt;
extern RemeasureManager remeasure;
#endif

// --- new configurable parameters for temprise absolute/blending ---
#ifndef TEMPRISE_ABS_MAX_DEPTH
#define TEMPRISE_ABS_MAX_DEPTH 8   // hard upper bound for memory
#endif

// Defaults (change at runtime if needed)
int temprise_abs_depth = 4;        // how many recent log entries to use
float temprise_balance = 0.7f;     // final = balance*relative + (1-balance)*absolute (0..1)

// recent log circular buffer (keeps last TEMPRISE_ABS_MAX_DEPTH entries)
static ChargeLogData recentChargeLogs[TEMPRISE_ABS_MAX_DEPTH];
static int recentChargeLogsCount = 0;     // number of valid entries currently stored (0..TEMPRISE_ABS_MAX_DEPTH)
static int recentChargeLogsHead = 0;      // next write position (0..TEMPRISE_ABS_MAX_DEPTH-1)

void pushRecentChargeLog(const ChargeLogData &entry) {
    recentChargeLogs[recentChargeLogsHead] = entry;
    recentChargeLogsHead = (recentChargeLogsHead + 1) % TEMPRISE_ABS_MAX_DEPTH;
    if (recentChargeLogsCount < TEMPRISE_ABS_MAX_DEPTH) {
        ++recentChargeLogsCount;
    }
}

int indexOfOldestEntry() {
    if (recentChargeLogsCount == 0) return -1;
    int idx = recentChargeLogsHead - recentChargeLogsCount;
    if (idx < 0) idx += TEMPRISE_ABS_MAX_DEPTH;
    return idx;
}

float computeAbsoluteTempRiseFromHistory(int depth) {
    if (depth <= 0) return NAN;
    if (recentChargeLogsCount == 0) return NAN;

    int use = depth;
    if (use > recentChargeLogsCount) use = recentChargeLogsCount;

    int oldestIndex = indexOfOldestEntry();
    if (oldestIndex < 0 || oldestIndex >= TEMPRISE_ABS_MAX_DEPTH) {
        return NAN;
    }

    if (use < recentChargeLogsCount) {
        oldestIndex = recentChargeLogsHead - use;
        while (oldestIndex < 0) oldestIndex += TEMPRISE_ABS_MAX_DEPTH;
        oldestIndex %= TEMPRISE_ABS_MAX_DEPTH;
    }

    const ChargeLogData &root = recentChargeLogs[oldestIndex];
    double T_sim = root.batteryTemperature;
    uint32_t prev_ts = root.timestamp;
    float final_ambient = 25 ;
    if (use == 1) return 0.0f;

    int idx = (oldestIndex + 1) % TEMPRISE_ABS_MAX_DEPTH;
    for (int i = 1; i < use; ++i) {
        const ChargeLogData &e = recentChargeLogs[idx];
        uint32_t ts = e.timestamp;
        uint32_t dt_ms = ts - prev_ts;
        if (dt_ms == 0) {
          dt_ms = 1U;
          ts = prev_ts + 1U;
         }

        float cur = e.current;
        if (!isfinite(cur) || cur < 0.0f) cur = 0.0f;

        float Rparam = regressedInternalResistancePairsIntercept;
        float vUnderLoad = e.voltage;
        if (!isfinite(vUnderLoad)) vUnderLoad = 0.0f;
        float vNoLoad = vUnderLoad ;

        float ambient = e.ambientTemperature;
        if (!isfinite(ambient)) ambient = 25.0f;
        final_ambient = ambient ;

        float local_unapplied = 0.0f; // History replay doesn't carry over unapplied energy across separate reevaluations in this simple model
        float theta_new = estimateTempDiff(
            vUnderLoad, vNoLoad, cur, Rparam, ambient, ts, prev_ts, (float)T_sim, &local_unapplied
        );

        if (isfinite(theta_new)) {
            double T_after = theta_new + ambient;
            double delta = T_after - T_sim;
            const float MAX_DELTA_PER_INTERVAL = 50.0f;
            if (delta > MAX_DELTA_PER_INTERVAL) delta = MAX_DELTA_PER_INTERVAL;
            if (delta < -MAX_DELTA_PER_INTERVAL) delta = -MAX_DELTA_PER_INTERVAL;
            T_sim += delta;
        }

        prev_ts = ts;
        idx = (idx + 1) % TEMPRISE_ABS_MAX_DEPTH;
    }

    float vs_ambient = (float)(T_sim - final_ambient);
    return vs_ambient;
}

void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay) {
    if (meas.active()) {
        return;
    }
    meas.reset();
    meas.testDuty = testDutyCycle;
    meas.stabilizationDelay = stabilization_delay;
    meas.unloadedDelay = unloaded_delay;
    applyDuty(0);
    meas.stateStart = millis();
    meas.state = MEAS_STOPLOAD_WAIT;
    meas.resultReady = false;
}

bool measurementStep() {
    if (meas.state == MEAS_IDLE || meas.state == MEAS_COMPLETE || meas.state == MEAS_ABORTED) return false;
    unsigned long now = millis();
    switch (meas.state) {
        case MEAS_STOPLOAD_WAIT:
        {
            applyDuty(0);
            if (now - meas.stateStart >= meas.unloadedDelay) {
                getThermistorReadings(meas.unloadedData.temp1, meas.unloadedData.temp2, meas.unloadedData.tempDiff,
                                      meas.unloadedData.t1_millivolts, meas.unloadedData.voltage, meas.unloadedData.current);
                meas.unloadedData.dutyCycle = 0;
                meas.unloadedData.timestamp = now;
                applyDuty(meas.testDuty);
                meas.stateStart = now;
                meas.state = MEAS_APPLY_LOAD;
            }
            break;
        };
        case MEAS_APPLY_LOAD:
        {
            if (now - meas.stateStart >= meas.stabilizationDelay) {
                getThermistorReadings(meas.loadedData.temp1, meas.loadedData.temp2, meas.loadedData.tempDiff,
                                      meas.loadedData.t1_millivolts, meas.loadedData.voltage, meas.loadedData.current);
                meas.loadedData.dutyCycle = meas.testDuty;
                meas.loadedData.timestamp = now;

                meas.result = MHElectrodeData();
                meas.result.unloadedVoltage = meas.unloadedData.voltage;
                meas.result.loadedVoltage = meas.loadedData.voltage;
                meas.result.current = meas.loadedData.current;
                meas.result.dutyCycle = meas.testDuty;
                meas.result.timestamp = now;
                meas.result.targetVoltage = meas.result.unloadedVoltage + (meas.result.loadedVoltage - meas.result.unloadedVoltage) * MH_ELECTRODE_RATIO;
                meas.result.voltageDifference = meas.result.loadedVoltage - meas.result.targetVoltage;

                if (meas.result.current < MEASURABLE_CURRENT_THRESHOLD) {
                    meas.result.current = estimateCurrent(meas.testDuty);
                }
                meas.resultReady = true;
                meas.state = MEAS_COMPLETE;
                applyDuty(0);
            }
            break;
        }
        default: break;
    }
    return meas.active();
}

bool fetchMeasurementResult(MHElectrodeData &out) {
    if (!meas.resultReady) return false;
    out = meas.result;
    meas.resultReady = false;
    return true;
}

void abortMeasurement() {
    if (meas.active()) {
        meas.state = MEAS_ABORTED;
        meas.resultReady = false;
        applyDuty(0);
    }
}

void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval) {
    findOpt = FindOptManager();
    findOpt.active = true;
    findOpt.maxDC = (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) ? MAX_CHARGE_DUTY_CYCLE : maxChargeDutyCycle;
    findOpt.lowDC = max(MIN_CHARGE_DUTY_CYCLE, (int)suggestedStartDutyCycle);
    findOpt.highDC = findOpt.maxDC;
    findOpt.optimalDC = findOpt.lowDC;
    findOpt.closestVoltageDifference = 1000.0f;
    findOpt.cache.reserve(MAX_RESISTANCE_POINTS);
    findOpt.phase = FIND_INIT_HIGHDC;
    findOpt.isReevaluation = isReeval;
    startMHElectrodeMeasurement(findOpt.highDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
}

bool findOptimalChargingDutyCycleStepAsync() {
    if (!findOpt.active) return false;
    measurementStep();
    if (findOpt.phase == FIND_INIT_HIGHDC) {
        if (meas.resultReady) {
            MHElectrodeData dataHigh;
            if (fetchMeasurementResult(dataHigh)) {
                findOpt.initialUnloadedVoltage = dataHigh.unloadedVoltage;
                findOpt.targetVoltage = findOpt.initialUnloadedVoltage + (dataHigh.loadedVoltage - dataHigh.unloadedVoltage) * MH_ELECTRODE_RATIO;
                if (dataHigh.current > 0.01f) {
                    float internalResistanceLUInitial = (findOpt.initialUnloadedVoltage - dataHigh.loadedVoltage) / dataHigh.current;
                    storeOrAverageResistanceData(dataHigh.current, std::fabs(internalResistanceLUInitial),
                                                 internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(dataHigh);
                findOpt.optimalDC = max(MIN_CHARGE_DUTY_CYCLE, findOpt.lowDC);
                findOpt.phase = FIND_BINARY_PREPARE;
            }
        }
        return true;
    }
    if (findOpt.phase == RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE) {
        int dc;
        if (findOpt.exploratory_measurement_phase == 0) {
            dc = findOpt.lowDC - 10;
            if (dc < MIN_CHARGE_DUTY_CYCLE) dc = MIN_CHARGE_DUTY_CYCLE;
        } else {
            dc = findOpt.highDC + 10;
            if (dc > MAX_CHARGE_DUTY_CYCLE) dc = MAX_CHARGE_DUTY_CYCLE;
        }
        startMHElectrodeMeasurement(dc, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_WAIT;
        return true;
    }
    if (findOpt.phase == RE_EVAL_EXPLORATORY_MEASUREMENT_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData result;
            if (fetchMeasurementResult(result)) {
                if (result.current > 0.001f) {
                    for (const auto& cached : findOpt.cache) {
                        if (std::fabs(result.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                            float internalResistancePair = (cached.loadedVoltage - result.loadedVoltage) / (result.current - cached.current);
                            float higherCurrent = max(result.current, cached.current);
                            storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                         internalResistanceDataPairs, resistanceDataCountPairs);
                        }
                    }
                }
                findOpt.cache.push_back(result);
                findOpt.exploratory_measurement_phase++;
                if (findOpt.exploratory_measurement_phase >= 2) {
                    findOpt.phase = RE_EVAL_FINISH;
                } else {
                    findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
                }
            }
        }
        return true;
    }
    if (findOpt.phase == RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE) {
        if (findOpt.outlier_measurement_index >= (int)findOpt.outliers.size()) {
            findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
            return true;
        }
        const OutlierInfo& outlier = findOpt.outliers[findOpt.outlier_measurement_index];
        startRemeasure(outlier.current);
        for (int i = outlier.original_index; i < resistanceDataCountPairs - 1; ++i) {
            internalResistanceDataPairs[i][0] = internalResistanceDataPairs[i + 1][0];
            internalResistanceDataPairs[i][1] = internalResistanceDataPairs[i + 1][1];
        }
        resistanceDataCountPairs--;
        findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_WAIT;
        return true;
    }
    if (findOpt.phase == RE_EVAL_CORRECTIVE_MEASUREMENT_WAIT) {
        if (!remeasureStep()) {
            findOpt.outlier_measurement_index++;
            findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE;
        }
        return true;
    }
    if (findOpt.phase == RE_EVAL_FINISH) {
        distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
        distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
        if (resistanceDataCount >= 2) {
            performLinearRegression(internalResistanceData, resistanceDataCount,
                                        regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
        if (resistanceDataCountPairs >= 2) {
            performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                        regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
        findOpt.phase = FIND_COMPLETE;
        findOpt.active = false;
        return false;
    }
    if (findOpt.phase == RE_EVAL_START) {
        findOpt.outliers.clear();
        findOpt.outlier_measurement_index = 0;
        findOpt.exploratory_measurement_phase = 0;
        findOpt.phase = RE_EVAL_DETECT_OUTLIERS;
    }
    if (findOpt.phase == RE_EVAL_DETECT_OUTLIERS) {
        if (resistanceDataCountPairs < 5) {
            findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
            return true;
        }
        float mean = 0.0f;
        float std_dev = 0.0f;
        for (int i = 0; i < resistanceDataCountPairs; ++i) {
            mean += internalResistanceDataPairs[i][1];
        }
        mean /= resistanceDataCountPairs;
        for (int i = 0; i < resistanceDataCountPairs; ++i) {
            std_dev += pow(internalResistanceDataPairs[i][1] - mean, 2);
        }
        std_dev = sqrt(std_dev / resistanceDataCountPairs);
        for (int i = 0; i < resistanceDataCountPairs; ++i) {
            if (fabs(internalResistanceDataPairs[i][1] - mean) > 2 * std_dev) {
                OutlierInfo outlier;
                outlier.original_index = i;
                outlier.current = internalResistanceDataPairs[i][0];
                outlier.resistance = internalResistanceDataPairs[i][1];
                findOpt.outliers.push_back(outlier);
            }
        }
        if (findOpt.outliers.empty()) {
            findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
        } else {
            std::sort(findOpt.outliers.begin(), findOpt.outliers.end(), [](const OutlierInfo& a, const OutlierInfo& b) {
                return a.original_index > b.original_index;
            });
            findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE;
        }
        return true;
    }
    if (findOpt.phase == FIND_BINARY_PREPARE) {
        if (findOpt.highDC - findOpt.lowDC <= CHARGE_CURRENT_STEP * 2) {
            if (findOpt.isReevaluation) {
                findOpt.phase = RE_EVAL_START;
                return true;
            }
            distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
            if (resistanceDataCount >= 2) {
                performLinearRegression(internalResistanceData, resistanceDataCount,
                                            regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
            }
            if (resistanceDataCountPairs >= 2) {
                performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                            regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
            }
            if (findOpt.optimalDC < MIN_CHARGE_DUTY_CYCLE) findOpt.optimalDC = findOpt.lowDC;
            startMHElectrodeMeasurement(findOpt.optimalDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            findOpt.phase = FIND_FINAL_WAIT;
            return true;
        }
        int midDC = (findOpt.lowDC + findOpt.highDC) / 2;
        startMHElectrodeMeasurement(midDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = FIND_BINARY_WAIT;
        return true;
    }
    if (findOpt.phase == FIND_BINARY_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData cur;
            if (fetchMeasurementResult(cur)) {
                if (cur.current < MEASURABLE_CURRENT_THRESHOLD) {
                    cur.current = estimateCurrent(cur.dutyCycle);
                }
                if (cur.current > 0.001f) {
                    float internalResistanceLU = (cur.unloadedVoltage - cur.loadedVoltage) / cur.current;
                    storeOrAverageResistanceData(cur.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(cur);
                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(cur.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistancePair = (cached.loadedVoltage - cur.loadedVoltage) / (cur.current - cached.current);
                        float higherCurrent = max(cur.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                }
                if (cur.loadedVoltage < findOpt.targetVoltage) {
                    findOpt.lowDC = cur.dutyCycle;
                } else {
                    findOpt.highDC = cur.dutyCycle;
                }
                if (fabs(cur.loadedVoltage - findOpt.targetVoltage) < findOpt.closestVoltageDifference) {
                    findOpt.closestVoltageDifference = fabs(cur.loadedVoltage - findOpt.targetVoltage);
                    findOpt.optimalDC = cur.dutyCycle;
                }
                findOpt.phase = FIND_BINARY_PREPARE;
            }
        }
        return true;
    }
    if (findOpt.phase == FIND_FINAL_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData finalData;
            if (fetchMeasurementResult(finalData)) {
                if (finalData.current < MEASURABLE_CURRENT_THRESHOLD) {
                    finalData.current = estimateCurrent(finalData.dutyCycle);
                }
                if (finalData.current > 0.01f) {
                    float internalResistanceLU = (finalData.unloadedVoltage - finalData.loadedVoltage) / finalData.current;
                    storeOrAverageResistanceData(finalData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(finalData);
                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(finalData.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistancePair = (cached.loadedVoltage - finalData.loadedVoltage) / (finalData.current - cached.current);
                        float higherCurrent = max(finalData.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                }
                distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
                distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
                if (resistanceDataCount >= 2) {
                    performLinearRegression(internalResistanceData, resistanceDataCount,
                                                regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                }
                if (resistanceDataCountPairs >= 2) {
                    performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                                regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                }
                findOpt.targetVoltage = findOpt.initialUnloadedVoltage +
                                        (finalData.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;
                cachedOptimalDuty = findOpt.optimalDC;
                findOpt.active = false;
                findOpt.phase = FIND_COMPLETE;
                return false;
            }
        }
        return true;
    }
    return true;
}

void startRemeasure(float targetCurrent) {
    remeasure = RemeasureManager();
    remeasure.active = true;
    remeasure.targetCurrent = targetCurrent;
    int predicted_dc = estimateDutyCycleForCurrent(targetCurrent);
    remeasure.lowDC = max(MIN_CHARGE_DUTY_CYCLE, (int)(predicted_dc - 20));
    remeasure.highDC = min(MAX_CHARGE_DUTY_CYCLE, (int)(predicted_dc + 20));
    remeasure.phase = REMEASURE_BINARY_SEARCH_PREPARE;
}

bool remeasureStep() {
    if (!remeasure.active) return false;
    measurementStep();
    switch (remeasure.phase) {
        case REMEASURE_BINARY_SEARCH_PREPARE: {
            if (remeasure.highDC - remeasure.lowDC <= CHARGE_CURRENT_STEP * 2) {
                remeasure.phase = REMEASURE_COMPLETE;
                return false;
            }
            int midDC = (remeasure.lowDC + remeasure.highDC) / 2;
            startMHElectrodeMeasurement(midDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            remeasure.phase = REMEASURE_BINARY_SEARCH_WAIT;
            return true;
        }
        case REMEASURE_BINARY_SEARCH_WAIT: {
            if (meas.resultReady) {
                MHElectrodeData result;
                if (fetchMeasurementResult(result)) {
                    if (result.current > 0.001f) {
                        float internalResistanceLU = (result.unloadedVoltage - result.loadedVoltage) / result.current;
                        storeOrAverageResistanceData(result.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                        bubbleSort(internalResistanceData, resistanceDataCount);
                        for (const auto& cached : findOpt.cache) {
                            if (std::fabs(result.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                                float internalResistancePair = (cached.loadedVoltage - result.loadedVoltage) / (result.current - cached.current);
                                float higherCurrent = max(result.current, cached.current);
                                storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                             internalResistanceDataPairs, resistanceDataCountPairs);
                                bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                            }
                        }
                    }
                    findOpt.cache.push_back(result);
                    if (result.current < remeasure.targetCurrent) {
                        remeasure.lowDC = result.dutyCycle;
                    } else {
                        remeasure.highDC = result.dutyCycle;
                    }
                    remeasure.phase = REMEASURE_BINARY_SEARCH_PREPARE;
                }
            }
            return true;
        }
        case REMEASURE_COMPLETE:
            remeasure.active = false;
            return false;
        default: return true;
    }
}

static float g_unappliedEnergy_J = 0.0f;
static float g_internalReleaseTau_s = 60.0f;

static float computeDissipatedPower(float vUnderLoad, float vNoLoad, float current, float Rparam) {
  const float epsI = 1e-9f;
  float P_v = 0.0f;
  if (current > epsI) {
    float vDrop = vNoLoad - vUnderLoad;
    if (vDrop > 1e-6f) P_v = current * vDrop;
  }
  float P_r = 0.0f;
  if (Rparam > 0.0f) P_r = current * current * Rparam;
  if (P_v > 0.0f && P_r > 0.0f) return 0.5f * (P_v + P_r);
  return (P_v > 0.0f) ? P_v : P_r;
}

static float thermalConductance_W_per_K(float area, float h, float emissivity, float T_ambientK) {
  return h * area + 4.0f * emissivity * STEFAN_BOLTZMANN * area * powf(T_ambientK, 3.0f);
}

float estimateTempDiff(
  float voltageUnderLoad, float voltageNoLoad, float current, float internalResistanceParam,
  float ambientTempC, uint32_t currentTime, uint32_t lastChargeEvaluationTime, float BatteryTempC,
  float* unappliedEnergy_J,
  float cellMassKg, float specificHeat, float area, float convectiveH, float emissivity
) {
  uint32_t dt_ms = (uint32_t)(currentTime - lastChargeEvaluationTime);
  float dt_s = (float)dt_ms * 0.001f;
  if (dt_s <= 0.0f) return BatteryTempC - ambientTempC;

  float P = computeDissipatedPower(voltageUnderLoad, voltageNoLoad, current, internalResistanceParam);
  float T_amb_K = ambientTempC + 273.15f;
  float G = thermalConductance_W_per_K(area, convectiveH, emissivity, T_amb_K);
  float Cth = cellMassKg * specificHeat;
  const float tiny = 1e-12f;
  float theta0 = BatteryTempC - ambientTempC;
  float E_generated_J = P * dt_s;
  float E_released_J = 0.0f;
  if (unappliedEnergy_J && *unappliedEnergy_J > 0.0f) {
    float tau_rel = (g_internalReleaseTau_s > tiny) ? g_internalReleaseTau_s : tiny;
    float frac = 1.0f - expf(-dt_s / tau_rel);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    E_released_J = *unappliedEnergy_J * frac;
    *unappliedEnergy_J -= E_released_J;
    if (*unappliedEnergy_J < 0.0f) *unappliedEnergy_J = 0.0f;
  }
  float E_input_total_J = E_generated_J + E_released_J;
  if (G <= tiny || Cth <= tiny) {
    float deltaT = E_input_total_J / ( (Cth <= tiny) ? 1.0f : Cth );
    return theta0 + deltaT;
  }
  float P_avg = E_input_total_J / dt_s;
  float theta_ss = P_avg / G;
  float tau = Cth / G;
  float theta_new = theta_ss + (theta0 - theta_ss) * expf(-dt_s / tau);
  return theta_new;
}

bool chargeBattery() {
    static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    unsigned long now = millis();

    if (chargingState != CHARGE_IDLE && chargingState != CHARGE_STOPPED && (now - chargingStartTime >= TOTAL_TIMEOUT)) {
        chargingState = CHARGE_STOPPED;
    }

    switch (chargingState) {
        case CHARGE_IDLE: {
            chargingStartTime = now;
            eval_mAh_snapshot = (float)mAh_charged;
            eval_time_snapshot = now;
            reeval_active = false;
            lastChargeEvaluationTime = now;
            overtemp_trip_counter = 0;
            lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
            currentRampTarget = 0.0f;

            MeasurementData initialData;
            getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                                  initialData.t1_millivolts, initialData.voltage, initialData.current);
            ChargeLogData startLog;
            startLog.timestamp = now;
            startLog.current = initialData.current;
            startLog.voltage = initialData.voltage;
            startLog.ambientTemperature = (float)initialData.temp1;
            startLog.batteryTemperature = (float)initialData.temp2;
            startLog.dutyCycle = MAX_CHARGE_DUTY_CYCLE;
            startLog.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
            startLog.internalResistancePairs = regressedInternalResistancePairsIntercept;
            logChargeData(startLog);
            pushRecentChargeLog(startLog);
            startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE, false);
            chargingState = CHARGE_FIND_OPT;
            break;
        }
        case CHARGE_FIND_OPT: {
            if (!findOptimalChargingDutyCycleStepAsync()) {
                int appliedDC = findOpt.optimalDC;
                if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
                if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;
                applyDuty(appliedDC);
                lastOptimalDutyCycle = dutyCycle;
                lastChargeEvaluationTime = now;
                lastReeval_delta_mAh = (float)(mAh_charged - reeval_start_mAh);
                lastReeval_duration_ms = (now > reeval_start_ms) ? (now - reeval_start_ms) : 0;
                double reeval_duration_h = lastReeval_duration_ms / 3600000.0f;
                double reeval_Ah = lastReeval_delta_mAh / 1000.0f;
                if (reeval_duration_h > 0.0f) {
                    lastReeval_avgCurrent_A = (float)(reeval_Ah / reeval_duration_h);
                } else {
                    lastReeval_avgCurrent_A = 0.0f;
                }
                reeval_active = false;
                chargingState = CHARGE_MONITOR;
            }
            break;
        }
        case CHARGE_MONITOR: {
            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);
            if (current > currentRampTarget) {
              if (dutyCycle > 0){
                applyDuty(dutyCycle - 1);
              }
            }
            if (now - eval_time_snapshot >= CHARGE_EVALUATION_INTERVAL_MS) {
                currentRampTarget += maximumCurrent * 0.10f;
                if (currentRampTarget > maximumCurrent) currentRampTarget = maximumCurrent;
                double delta_mAh = mAh_charged - eval_mAh_snapshot;
                unsigned long delta_ms = now - eval_time_snapshot;
                double delta_h = delta_ms / 3600000.0f;
                double avgCurrentA = 0.0f;
                if (delta_h > 0.0f) avgCurrentA = (delta_mAh / 1000.0f) / delta_h;

                float tempRise_relative = estimateTempDiff(
                    voltage, voltage, (float)avgCurrentA, regressedInternalResistancePairsIntercept,
                    (float)temp1, now, eval_time_snapshot, (float)temp2, &g_unappliedEnergy_J
                );
                ChargeLogData entry;
                entry.timestamp = now;
                entry.current = (float)avgCurrentA;
                entry.voltage = voltage;
                entry.ambientTemperature = (float)temp1;
                entry.batteryTemperature = (float)temp2;
                entry.dutyCycle = dutyCycle;
                entry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
                entry.internalResistancePairs = regressedInternalResistancePairsIntercept;
                logChargeData(entry);
                pushRecentChargeLog(entry);

                float temprise_absolute = NAN;
                if (recentChargeLogsCount >= 1) {
                    int depthToUse = temprise_abs_depth;
                    if (depthToUse > recentChargeLogsCount) depthToUse = recentChargeLogsCount;
                    temprise_absolute = computeAbsoluteTempRiseFromHistory(depthToUse);
                }
                if (isnan(temprise_absolute)) temprise_absolute = tempRise_relative;
                if (temprise_absolute < 0.0f && fabs(temprise_absolute) < 1e-4f) temprise_absolute = 0.0f;
                if (tempRise_relative < 0.0f && fabs(tempRise_relative) < 1e-4f) tempRise_relative = 0.0f;
                float finalTempRise = temprise_balance * tempRise_relative + (1.0f - temprise_balance) * temprise_absolute;

                eval_mAh_snapshot = (float)mAh_charged;
                eval_time_snapshot = now;
                lastChargeEvaluationTime = now;
                if (currentRampTarget >= maximumCurrent) {
                    if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD + finalTempRise)) {
                        if (overtemp_trip_counter++ >= OVERTEMP_TRIP_TRESHOLD) {
                            overtemp_trip_counter = 0;
                            chargingState = CHARGE_STOPPED;
                        }
                    }
                }
                if (chargingState == CHARGE_MONITOR) {
                    reeval_active = true;
                    reeval_start_mAh = (float)mAh_charged;
                    reeval_start_ms = now;
                    int suggestedStartDutyCycle = min( (int)(0.5 * lastOptimalDutyCycle),MAX_CHARGE_DUTY_CYCLE);
                    int suggestedEndDutyCycle   = max( (int)(2 * lastOptimalDutyCycle),MIN_CHARGE_DUTY_CYCLE);
                    startFindOptimalManagerAsync(suggestedEndDutyCycle, suggestedStartDutyCycle, true);
                    chargingState = CHARGE_FIND_OPT;
                }
            }
            break;
        }
        case CHARGE_STOPPED: {
            applyDuty(0);
            return false;
        }
        default: break;
    }
    return true;
}

void startCharging() {
    if (currentAppState != APP_STATE_CHARGING) {
        currentAppState = APP_STATE_CHARGING;
        chargingState = CHARGE_IDLE;
    }
}

void stopCharging() {
  if (currentAppState == APP_STATE_CHARGING && chargingState != CHARGE_STOPPED) {
    chargingState = CHARGE_STOPPED;
    applyDuty(0);
  }
}
