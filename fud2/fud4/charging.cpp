#include "charging.h"
#include "definitions.h"
#include "logging.h"
#include "internal_resistance.h"


unsigned long chargingStartTime = 0;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;
float currentRampTarget = 0.0f;

static uint8_t outgassing_trip_counter = 0;
static bool outgassingTriggered = false;
static unsigned long lastHousekeepTime = 0;

float getAverageResistanceNearCurrent(float cur, float data[][2], int count) {
    if (count <= 0) return 0.18f;
    float sumR = 0.0f;
    int matchingCount = 0;
    float tolerance = 0.15f;
    for (int i = 0; i < count; ++i) {
        if (std::fabs(data[i][0] - cur) <= tolerance) {
            sumR += data[i][1];
            matchingCount++;
        }
    }
    if (matchingCount > 0) {
        return sumR / (float)matchingCount;
    }
    tolerance = 0.35f;
    for (int i = 0; i < count; ++i) {
        if (std::fabs(data[i][0] - cur) <= tolerance) {
            sumR += data[i][1];
            matchingCount++;
        }
    }
    if (matchingCount > 0) {
        return sumR / (float)matchingCount;
    }
    for (int i = 0; i < count; ++i) {
        sumR += data[i][1];
    }
    return sumR / (float)count;
}

// Pulse charge current mean accumulation tracking
static double pulseCurrentSum = 0.0;
static uint32_t pulseCurrentSamples = 0;

// Global thermal tracking and derivative variables
float recoveredAmbientTemp = 25.0f;
float recoveredBatteryTemp = 25.0f;
double prev_t1 = -1.0;
double prev_t2 = -1.0;
double t1_deriv = 0.0;
double t2_deriv = 0.0;
float predictedTempTrack = 25.0f;
unsigned long pulseCycleStartTime = 0;
unsigned long lastLogTime = 0;

// Divergence tracking for physical P_residual model
double prev_divergence_m = 0.0;
bool prev_divergence_m_set = false;
double dD_dt_smooth = 0.0;
double P_residual_slow = 0.0;

// Dynamic self-tuning outgassing threshold and integration variables
float residualEnergy_J = 0.0f;
float dynamicP_threshold = 0.15f;
float dynamicE_threshold = 10.0f;
bool baseline_calibrated = false;
float baseline_pres_samples[120];
int baseline_pres_count = 0;
float baseline_mean = 0.0f;
float baseline_std = 0.0f;

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

// --- recent log circular buffer ---
#ifndef TEMPRISE_ABS_MAX_DEPTH
#define TEMPRISE_ABS_MAX_DEPTH 8
#endif

int temprise_abs_depth = 4;
float temprise_balance = 0.7f;
static ChargeLogData recentChargeLogs[TEMPRISE_ABS_MAX_DEPTH];
static int recentChargeLogsCount = 0;
static int recentChargeLogsHead = 0;

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
    if (depth <= 0 || recentChargeLogsCount == 0) return NAN;
    int use = (depth > recentChargeLogsCount) ? recentChargeLogsCount : depth;
    int oldestIndex = (recentChargeLogsHead - use + TEMPRISE_ABS_MAX_DEPTH) % TEMPRISE_ABS_MAX_DEPTH;

    const ChargeLogData &root = recentChargeLogs[oldestIndex];
    double T_sim = root.batteryTemperature;
    uint32_t prev_ts = root.timestamp;
    float final_ambient = 25;
    if (use == 1) return 0.0f;

    int idx = (oldestIndex + 1) % TEMPRISE_ABS_MAX_DEPTH;
    for (int i = 1; i < use; ++i) {
        const ChargeLogData &e = recentChargeLogs[idx];
        uint32_t ts = e.timestamp;
        uint32_t dt_ms = ts - prev_ts;
        if (ts <= prev_ts) { ts = prev_ts + 1; dt_ms = 1; }

        float cur = e.current;
        if (!std::isfinite(cur) || cur < 0.0f) cur = 0.0f;
        float Rparam = getAverageResistanceNearCurrent(cur, internalResistanceDataPairs, resistanceDataCountPairs);
        if (Rparam < 0.01f) Rparam = 0.01f;
        if (Rparam > 5.0f) Rparam = 5.0f;
        float vUnderLoad = e.voltage;
        float ambient = e.ambientTemperature;
        if (!std::isfinite(ambient)) ambient = 25.0f;
        final_ambient = ambient;

        float local_unapplied = 0.0f;
        float theta_new = estimateTempDiff(vUnderLoad, vUnderLoad, cur, Rparam, ambient, ts, prev_ts, (float)T_sim, &local_unapplied);
        if (std::isfinite(theta_new)) {
            double T_after = theta_new + ambient;
            double delta = T_after - T_sim;
            if (delta > 50.0) delta = 50.0;
            if (delta < -50.0) delta = -50.0;
            T_sim += delta;
        }
        prev_ts = ts;
        idx = (idx + 1) % TEMPRISE_ABS_MAX_DEPTH;
    }
    return (float)(T_sim - final_ambient);
}

void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay) {
    if (meas.active()) return;
    meas.reset();
    meas.testDuty = (uint8_t)testDutyCycle;
    ElectrodeParameters electrodeSnap = getElectrodeParametersSnapshot();
    if (stabilization_delay == STABILIZATION_DELAY_MS && electrodeSnap.evaluated) {
        stabilization_delay = electrodeSnap.adaptiveDelayMs;
    }
    meas.stabilizationDelay = stabilization_delay;
    meas.unloadedDelay = unloaded_delay;
    applyDuty(0);
    meas.stateStart = millis();
    meas.state = MEAS_STOPLOAD_WAIT;
}

bool measurementStep() {
    if (meas.state == MEAS_IDLE || meas.state == MEAS_COMPLETE || meas.state == MEAS_ABORTED) return false;
    unsigned long now = millis();
    switch (meas.state) {
        case MEAS_STOPLOAD_WAIT:
            if (now - meas.stateStart >= meas.unloadedDelay) {
                getThermistorReadings(meas.unloadedData.temp1, meas.unloadedData.temp2, meas.unloadedData.tempDiff,
                                      meas.unloadedData.t1_millivolts, meas.unloadedData.voltage, meas.unloadedData.current);
                meas.unloadedData.dutyCycle = 0;
                meas.unloadedData.timestamp = (uint32_t)now;
                applyDuty(meas.testDuty);
                meas.stateStart = now;
                meas.state = MEAS_APPLY_LOAD;
            }
            break;
        case MEAS_APPLY_LOAD:
            if (now - meas.stateStart >= meas.stabilizationDelay) {
                getThermistorReadings(meas.loadedData.temp1, meas.loadedData.temp2, meas.loadedData.tempDiff,
                                      meas.loadedData.t1_millivolts, meas.loadedData.voltage, meas.loadedData.current);
                meas.loadedData.dutyCycle = meas.testDuty;
                meas.loadedData.timestamp = (uint32_t)now;
                meas.result.unloadedVoltage = meas.unloadedData.voltage;
                meas.result.loadedVoltage = meas.loadedData.voltage;
                meas.result.current = meas.loadedData.current;
                meas.result.dutyCycle = meas.testDuty;
                meas.result.timestamp = (uint32_t)now;
                meas.result.targetVoltage = meas.result.unloadedVoltage + (meas.result.loadedVoltage - meas.unloadedData.voltage) * MH_ELECTRODE_RATIO;
                meas.result.voltageDifference = meas.result.loadedVoltage - meas.result.targetVoltage;
                if (meas.result.current < MEASURABLE_CURRENT_THRESHOLD) meas.result.current = estimateCurrent(meas.testDuty);
                meas.resultReady = true;
                meas.state = MEAS_COMPLETE;
                applyDuty(0);
            }
            break;
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
    if (meas.active()) { meas.state = MEAS_ABORTED; applyDuty(0); }
}

void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval) {
    findOpt.active = true;
    findOpt.maxDC = (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) ? MAX_CHARGE_DUTY_CYCLE : maxChargeDutyCycle;
    findOpt.lowDC = std::max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
    findOpt.highDC = findOpt.maxDC;
    findOpt.optimalDC = findOpt.lowDC;
    findOpt.closestVoltageDifference = 1000.0f;
    findOpt.targetVoltage = 0.0f;
    findOpt.initialUnloadedVoltage = 0.0f;
    findOpt.cache.clear();
    findOpt.phase = FIND_INIT_HIGHDC;
    findOpt.isReevaluation = isReeval;
    findOpt.outliers.clear();
    findOpt.outlier_measurement_index = 0;
    findOpt.exploratory_measurement_phase = 0;
    startMHElectrodeMeasurement(findOpt.highDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
}

bool findOptimalChargingDutyCycleStepAsync() {
    if (!findOpt.active) return false;
    measurementStep();
    if (findOpt.phase == FIND_INIT_HIGHDC && meas.resultReady) {
        MHElectrodeData dataHigh;
        if (fetchMeasurementResult(dataHigh)) {
            findOpt.initialUnloadedVoltage = dataHigh.unloadedVoltage;
            findOpt.targetVoltage = findOpt.initialUnloadedVoltage + (dataHigh.loadedVoltage - dataHigh.unloadedVoltage) * MH_ELECTRODE_RATIO;
            if (dataHigh.current > 0.01f) {
                float irLU = (findOpt.initialUnloadedVoltage - dataHigh.loadedVoltage) / dataHigh.current;
                WEB_LOCK();
                storeOrAverageResistanceData(dataHigh.current, std::fabs(irLU), internalResistanceData, resistanceDataCount);
                bubbleSort(internalResistanceData, resistanceDataCount);
                WEB_UNLOCK();
            }
            findOpt.cache.push_back(dataHigh);
            findOpt.optimalDC = std::max(MIN_CHARGE_DUTY_CYCLE, findOpt.lowDC);
            findOpt.phase = FIND_BINARY_PREPARE;
        }
        return true;
    }
    if (findOpt.phase == RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE) {
        RePointBuffer explPoints;
        allocateReevaluationCandidates(explPoints, 2);
        int dc = findOpt.lowDC;
        if (!explPoints.empty()) {
            dc = explPoints[findOpt.exploratory_measurement_phase % explPoints.size()].duty;
        } else {
            dc = (findOpt.exploratory_measurement_phase == 0) ? std::max(MIN_CHARGE_DUTY_CYCLE, findOpt.lowDC - 10) : std::min(MAX_CHARGE_DUTY_CYCLE, findOpt.highDC + 10);
        }
        startMHElectrodeMeasurement(dc, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_WAIT;
        return true;
    }
    if (findOpt.phase == RE_EVAL_EXPLORATORY_MEASUREMENT_WAIT && meas.resultReady) {
        MHElectrodeData result;
        if (fetchMeasurementResult(result)) {
            if (result.current > 0.001f) {
                WEB_LOCK();
                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(result.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float irPair = (cached.loadedVoltage - result.loadedVoltage) / (result.current - cached.current);
                        storeOrAverageResistanceData(std::max(result.current, cached.current), std::fabs(irPair), internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                }
                WEB_UNLOCK();
            }
            findOpt.cache.push_back(result);
            if (++findOpt.exploratory_measurement_phase >= 2) findOpt.phase = RE_EVAL_FINISH;
            else findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
        }
        return true;
    }
    if (findOpt.phase == RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE) {
        if (findOpt.outlier_measurement_index >= (int)findOpt.outliers.size()) { findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE; return true; }
        const OutlierInfo& outlier = findOpt.outliers[findOpt.outlier_measurement_index];
        startRemeasure(outlier.current);
        for (int i = outlier.original_index; i < resistanceDataCountPairs - 1; ++i) internalResistanceDataPairs[i][0] = internalResistanceDataPairs[i + 1][0], internalResistanceDataPairs[i][1] = internalResistanceDataPairs[i + 1][1];
        resistanceDataCountPairs--;
        findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_WAIT;
        return true;
    }
    if (findOpt.phase == RE_EVAL_CORRECTIVE_MEASUREMENT_WAIT && !remeasureStep()) { findOpt.outlier_measurement_index++; findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE; return true; }
    if (findOpt.phase == RE_EVAL_FINISH) {
        WEB_LOCK();
        distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
        distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
        if (resistanceDataCount >= 2) performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        if (resistanceDataCountPairs >= 2) performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        WEB_UNLOCK();
        findOpt.phase = FIND_COMPLETE; findOpt.active = false; return false;
    }
    if (findOpt.phase == RE_EVAL_START) { findOpt.outliers.clear(); findOpt.outlier_measurement_index = 0; findOpt.exploratory_measurement_phase = 0; findOpt.phase = RE_EVAL_DETECT_OUTLIERS; return true; }
    if (findOpt.phase == RE_EVAL_DETECT_OUTLIERS) {
        if (resistanceDataCountPairs < 5) { findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE; return true; }
        float mean = 0.0f, std_dev = 0.0f;
        for (int i = 0; i < resistanceDataCountPairs; ++i) mean += internalResistanceDataPairs[i][1];
        mean /= resistanceDataCountPairs;
        for (int i = 0; i < resistanceDataCountPairs; ++i) std_dev += pow(internalResistanceDataPairs[i][1] - mean, 2);
        std_dev = sqrt(std_dev / resistanceDataCountPairs);
        for (int i = 0; i < resistanceDataCountPairs; ++i) {
            if (fabs(internalResistanceDataPairs[i][1] - mean) > 2 * std_dev) {
                OutlierInfo o; o.original_index = i; o.current = internalResistanceDataPairs[i][0]; o.resistance = internalResistanceDataPairs[i][1]; findOpt.outliers.push_back(o);
            }
        }
        if (findOpt.outliers.empty()) findOpt.phase = RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE;
        else { std::sort(findOpt.outliers.begin(), findOpt.outliers.begin() + findOpt.outliers.size(), [](const OutlierInfo& a, const OutlierInfo& b) { return a.original_index > b.original_index; }); findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE; }
        return true;
    }
    if (findOpt.phase == FIND_BINARY_PREPARE) {
        if (findOpt.highDC - findOpt.lowDC <= CHARGE_CURRENT_STEP * 2) {
            if (findOpt.isReevaluation) { findOpt.phase = RE_EVAL_START; return true; }
            WEB_LOCK();
            distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
            if (resistanceDataCount >= 2) performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
            if (resistanceDataCountPairs >= 2) performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
            WEB_UNLOCK();
            startMHElectrodeMeasurement(findOpt.optimalDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            findOpt.phase = FIND_FINAL_WAIT; return true;
        }
        startMHElectrodeMeasurement((findOpt.lowDC + findOpt.highDC) / 2, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = FIND_BINARY_WAIT; return true;
    }
    if (findOpt.phase == FIND_BINARY_WAIT && meas.resultReady) {
        MHElectrodeData cur;
        if (fetchMeasurementResult(cur)) {
            if (cur.current < MEASURABLE_CURRENT_THRESHOLD) cur.current = estimateCurrent(cur.dutyCycle);
            if (cur.current > 0.001f) {
                float irLU = (cur.unloadedVoltage - cur.loadedVoltage) / cur.current;
                WEB_LOCK();
                storeOrAverageResistanceData(cur.current, std::fabs(irLU), internalResistanceData, resistanceDataCount);
                bubbleSort(internalResistanceData, resistanceDataCount);
                for (const auto& cached : findOpt.cache) if (std::fabs(cur.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                    float irP = (cached.loadedVoltage - cur.loadedVoltage) / (cur.current - cached.current);
                    storeOrAverageResistanceData(std::max(cur.current, cached.current), std::fabs(irP), internalResistanceDataPairs, resistanceDataCountPairs);
                }
                WEB_UNLOCK();
            }
            findOpt.cache.push_back(cur);
            if (cur.loadedVoltage < findOpt.targetVoltage) findOpt.lowDC = cur.dutyCycle; else findOpt.highDC = cur.dutyCycle;
            if (fabs(cur.loadedVoltage - findOpt.targetVoltage) < findOpt.closestVoltageDifference) { findOpt.closestVoltageDifference = fabs(cur.loadedVoltage - findOpt.targetVoltage); findOpt.optimalDC = cur.dutyCycle; }
            findOpt.phase = FIND_BINARY_PREPARE;
        }
        return true;
    }
    if (findOpt.phase == FIND_FINAL_WAIT && meas.resultReady) {
        MHElectrodeData finalData;
        if (fetchMeasurementResult(finalData)) {
            if (finalData.current < MEASURABLE_CURRENT_THRESHOLD) finalData.current = estimateCurrent(finalData.dutyCycle);
            if (finalData.current > 0.01f) {
                float irLU = (finalData.unloadedVoltage - finalData.loadedVoltage) / finalData.current;
                WEB_LOCK();
                storeOrAverageResistanceData(finalData.current, std::fabs(irLU), internalResistanceData, resistanceDataCount);
                bubbleSort(internalResistanceData, resistanceDataCount);
                for (const auto& cached : findOpt.cache) if (std::fabs(finalData.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                    float irP = (cached.loadedVoltage - finalData.loadedVoltage) / (finalData.current - cached.current);
                    storeOrAverageResistanceData(std::max(finalData.current, cached.current), std::fabs(irP), internalResistanceDataPairs, resistanceDataCountPairs);
                }
                WEB_UNLOCK();
            }
            WEB_LOCK();
            distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);
            if (resistanceDataCount >= 2) performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
            if (resistanceDataCountPairs >= 2) performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
            WEB_UNLOCK();
            cachedOptimalDuty = findOpt.optimalDC; findOpt.active = false; findOpt.phase = FIND_COMPLETE; return false;
        }
    }
    return true;
}

void startRemeasure(float targetCurrent) {
    remeasure.active = true;
    remeasure.targetCurrent = targetCurrent;
    int predicted = estimateDutyCycleForCurrent(targetCurrent);
    remeasure.lowDC = (uint8_t)std::max(MIN_CHARGE_DUTY_CYCLE, predicted - 20);
    remeasure.highDC = (uint8_t)std::min(MAX_CHARGE_DUTY_CYCLE, predicted + 20);
    int initialDuty = (remeasure.lowDC + remeasure.highDC) / 2;
    startMHElectrodeMeasurement(initialDuty, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
    remeasure.phase = REMEASURE_BINARY_SEARCH_WAIT;
}

bool remeasureStep() {
    if (!remeasure.active) return false;
    measurementStep();
    switch (remeasure.phase) {
        case REMEASURE_BINARY_SEARCH_PREPARE:
            if (remeasure.highDC - remeasure.lowDC <= CHARGE_CURRENT_STEP * 2) { remeasure.phase = REMEASURE_COMPLETE; return false; }
            startMHElectrodeMeasurement((remeasure.lowDC + remeasure.highDC) / 2, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            remeasure.phase = REMEASURE_BINARY_SEARCH_WAIT; return true;
        case REMEASURE_BINARY_SEARCH_WAIT:
            if (meas.resultReady) {
                MHElectrodeData res;
                if (fetchMeasurementResult(res)) {
                    if (res.current > 0.001f) {
                        float irLU = (res.unloadedVoltage - res.loadedVoltage) / res.current;
                    WEB_LOCK();
                        storeOrAverageResistanceData(res.current, std::fabs(irLU), internalResistanceData, resistanceDataCount);
                        bubbleSort(internalResistanceData, resistanceDataCount);
                        for (const auto& cached : findOpt.cache) if (std::fabs(res.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                            float irP = (cached.loadedVoltage - res.loadedVoltage) / (res.current - cached.current);
                            storeOrAverageResistanceData(std::max(res.current, cached.current), std::fabs(irP), internalResistanceDataPairs, resistanceDataCountPairs);
                        }
                    WEB_UNLOCK();
                    }
                    findOpt.cache.push_back(res);
                    if (res.current < remeasure.targetCurrent) remeasure.lowDC = res.dutyCycle; else remeasure.highDC = res.dutyCycle;
                    remeasure.phase = REMEASURE_BINARY_SEARCH_PREPARE;
                }
            }
            return true;
        case REMEASURE_COMPLETE: remeasure.active = false; return false;
        default: return true;
    }
}

float g_unappliedEnergy_J = 0.0f;
static float g_internalReleaseTau_s = 60.0f;

static float computeDissipatedPower(float vUnderLoad, float vNoLoad, float current, float Rparam) {
  // Prioritize pure ohmic heating (I^2 * R) as it accurately models Joule heat dissipation
  // while separating non-dissipative electrochemical polarization.
  if (Rparam > 1e-6f && current > 1e-6f) {
    return current * current * Rparam;
  }
  // Fall back to absolute voltage-drop power if Rparam is unavailable/uncalibrated
  float vDrop = std::fabs(vNoLoad - vUnderLoad);
  if (current > 1e-6f && vDrop > 1e-6f) {
    return current * vDrop;
  }
  return 0.0f;
}

static float thermalConductance_W_per_K(float area, float h, float emissivity, float T_ambientK) {
  return h * area + 4.0f * emissivity * STEFAN_BOLTZMANN * area * powf(T_ambientK, 3.0f);
}

void updateDynamicMaximumCurrent() {
    double t1, t2, td; float tmv, v, c;
    getThermistorReadings(t1, t2, td, tmv, v, c);
    float ambK = (float)t1 + 273.15f;
    float G = thermalConductance_W_per_K(DEFAULT_SURFACE_AREA_M2, estimatedConvectiveH, DEFAULT_EMISSIVITY, ambK);

    float R_int = regressedInternalResistancePairsIntercept;
    if (R_int < 0.01f) R_int = regressedInternalResistanceIntercept;
    if (R_int < 0.01f) R_int = 0.18f;

    float targetI = std::sqrt(1.0f * G / std::max(1e-4f, R_int));
    float maxCurrentLimit = 0.150f;
    maximumCurrent = std::min(maxCurrentLimit, std::max(0.010f, targetI));

    Serial.printf("Dynamic Max Current Updated: G=%.6f W/K, R_int=%.4f Ohms -> MaxCurrent=%.4f A (Clamped to 0.150A max)\n",
                  G, R_int, maximumCurrent);
}

float estimateTempDiff(float vL, float vN, float cur, float Rp, float ambC, uint32_t now, uint32_t last, float bC, float* uE, float mass, float spec, float area, float convH, float emiss) {
  if (!std::isfinite(vL)) vL = 1.2f;
  if (!std::isfinite(vN)) vN = 1.2f;
  if (!std::isfinite(cur) || cur < 0.0f) cur = 0.0f;
  if (!std::isfinite(Rp) || Rp <= 0.0f) Rp = 0.18f;
  if (!std::isfinite(ambC)) ambC = 25.0f;
  if (!std::isfinite(bC)) bC = ambC;

  uint32_t dt_ms = now - last;
  float dt_s = (float)dt_ms * 0.001f;
  if (dt_s <= 0.0f) return bC - ambC;

  // Numerical Stability Clamping: Guard against extremely tiny positive dt_s causing division blow-ups
  if (dt_s < 1e-4f) dt_s = 1e-4f;

  if (convH == DEFAULT_CONVECTIVE_H && std::isfinite(estimatedConvectiveH)) {
    convH = estimatedConvectiveH;
  }

  float P = computeDissipatedPower(vL, vN, cur, Rp);
  if (!std::isfinite(P)) P = 0.0f;

  float G = thermalConductance_W_per_K(area, convH, emiss, ambC + 273.15f);
  if (!std::isfinite(G) || G <= 1e-12f) G = 0.01f;

  float Cth = (mass == DEFAULT_CELL_MASS_KG && spec == DEFAULT_SPECIFIC_HEAT && std::isfinite(estimatedThermalCapacitance) && estimatedThermalCapacitance >= 5.0f && estimatedThermalCapacitance <= 50.0f)
              ? (float)estimatedThermalCapacitance
              : mass * spec;
  if (!std::isfinite(Cth) || Cth <= 1e-12f) Cth = 14.0f;

  float theta0 = bC - ambC;
  if (!std::isfinite(theta0)) theta0 = 0.0f;

  float E_gen = P * dt_s;
  float E_rel = 0.0f;
  if (uE && *uE > 0.0f && std::isfinite(*uE)) {
    float tau_rel = std::max(1e-12f, g_internalReleaseTau_s);
    float frac = 1.0f - expf(-dt_s / tau_rel);
    E_rel = *uE * frac; *uE -= E_rel;
    if (!std::isfinite(*uE) || *uE < 0.0f) *uE = 0.0f;
  }
  float E_total = E_gen + E_rel;
  float theta_ss = (E_total / dt_s) / G;
  if (!std::isfinite(theta_ss)) theta_ss = 0.0f;

  // Fit the thermal prediction model using the dynamically characterized estimatedTauThermal constant
  float tau = estimatedTauThermal;
  if (tau < 5.0f || !std::isfinite(tau)) {
      tau = Cth / G; // Fallback to theoretical if not characterized
  }

  // Numerical Stability Clamping: Guard against extremely large or near-infinite tau causing exponential division issues
  if (tau > 10000.0f || !std::isfinite(tau)) tau = 10000.0f;

  float res = theta_ss + (theta0 - theta_ss) * expf(-dt_s / tau);
  if (!std::isfinite(res)) return theta0;
  return res;
}

// Structured, smaller resolution IR test variables
struct StructuredIRTest {
    int step = 0;
    unsigned long stepStartTime = 0;
    float unloadedVoltage = 0.0f;
    float voltages[4] = {0.0f};
    float currents[4] = {0.0f};
    int duties[4] = {0};
    float calculatedIR = 0.15f;

    // Transient recording buffer for electrode parameter fitting
    static constexpr size_t TRANSIENT_MAX_SAMPLES = 64;
    float transient_time_s[TRANSIENT_MAX_SAMPLES];
    float transient_voltage_V[TRANSIENT_MAX_SAMPLES];
    float transient_current_A[TRANSIENT_MAX_SAMPLES];
    size_t transient_count = 0;
    unsigned long transient_start_ms = 0;
    float transient_unloaded_V = 0.0f;
    float transient_i_before = 0.0f;
    unsigned long last_sample_ms = 0;
};
static StructuredIRTest s_irTest;

ThermalHistoryBuffer s_thermalHistory;

// Structured IR Re-measurement Subsystem Structures
struct PulseIRRemeasure {
    bool active = false;
    int index = 0;
    int subStep = 0; // 0: wait unloaded, 1: wait loaded
    unsigned long stepStartTime = 0;
    float unloadedVoltage = 0.0f;
    float unloadedCurrent = 0.0f;
    RePointBuffer points;
};
static PulseIRRemeasure s_reMeasure;

void selectRandomRePoints() {
    s_reMeasure.points.clear();
    s_reMeasure.index = 0;
    s_reMeasure.subStep = 0;
    s_reMeasure.active = false;

    allocateReevaluationCandidates(s_reMeasure.points, PULSE_REMEASURE_BUDGET);

    if (!s_reMeasure.points.empty()) {
        s_reMeasure.active = true;
    }
}

bool chargeBattery() {
    unsigned long now = millis();
    if (chargingState != CHARGE_IDLE && chargingState != CHARGE_STOPPED && (now - chargingStartTime >= TOTAL_TIMEOUT)) chargingState = CHARGE_STOPPED;

    // Determine the pulse length based on estimated thermal time constant.
    // Let's target a combined pulse length equal to estimatedTauThermal / 2.0 (clamped).
    float pulseLengthS = estimatedTauThermal / 2.0f;
    if (pulseLengthS < MIN_PULSE_CYCLE_LENGTH_S) pulseLengthS = MIN_PULSE_CYCLE_LENGTH_S;
    if (pulseLengthS > MAX_PULSE_CYCLE_LENGTH_S) pulseLengthS = MAX_PULSE_CYCLE_LENGTH_S;
    unsigned long pulseLengthMs = (unsigned long)(pulseLengthS * 1000.0f);

    switch (chargingState) {
        case CHARGE_IDLE:
            chargingStartTime = now;
            updateDynamicMaximumCurrent();
            pulseCycleStartTime = now;
            eval_mAh_snapshot = (float)mAh_charged;
            eval_time_snapshot = now;
            overtemp_trip_counter = 0;
            outgassing_trip_counter = 0;
            outgassingTriggered = false;
            lastHousekeepTime = 0;
            WEB_LOCK();
            s_thermalHistory.clear();
            WEB_UNLOCK();
            lastLogTime = 0;
            g_unappliedEnergy_J = 0.0f;
            residualEnergy_J = 0.0f;
            dynamicP_threshold = 0.15f;
            dynamicE_threshold = 10.0f;
            baseline_calibrated = false;
            baseline_pres_count = 0;
            baseline_mean = 0.0f;
            baseline_std = 0.0f;
            {
                double t1, t2, td; float tmv, v, c; getThermistorReadings(t1, t2, td, tmv, v, c);
                predictedTempTrack = (float)t2;
                prev_t1 = t1;
                prev_t2 = t2;
                t1_deriv = 0.0;
                t2_deriv = 0.0;
                ChargeLogData s; s.timestamp = (uint32_t)now; s.current = c; s.voltage = v; s.ambientTemperature = (float)t1; s.batteryTemperature = (float)t2;
                s.internalResistanceLoadedUnloaded = getAverageResistanceNearCurrent(c, internalResistanceData, resistanceDataCount);
                s.internalResistancePairs = getAverageResistanceNearCurrent(c, internalResistanceDataPairs, resistanceDataCountPairs);
                s.threshold = predictedTempTrack - (float)t1;
                logChargeData(s); pushRecentChargeLog(s);
            }
            chargingState = CHARGE_PULSE_IR_TEST;
            s_irTest.step = 0;
            s_irTest.stepStartTime = now;
            applyDuty(0);
            break;

        case CHARGE_PULSE_IR_TEST:
            {
                // Structured, smaller resolution IR test (5 steps: unloaded, then 4 duties in increasing order)
                // Spend 1 second per step to capture voltage/current response.
                unsigned long stepElapsed = now - s_irTest.stepStartTime;
                double t1, t2, td; float tmv, v, cur; getThermistorReadings(t1, t2, td, tmv, v, cur);

                ElectrodeParameters electrodeSnap = getElectrodeParametersSnapshot();
                unsigned long reqStepDelay = electrodeSnap.evaluated ? electrodeSnap.adaptiveDelayMs : 1000;

                if (s_irTest.step == 0) {
                    if (stepElapsed >= reqStepDelay) {
                        s_irTest.unloadedVoltage = v;
                        s_irTest.step = 1;
                        s_irTest.stepStartTime = now;
                        s_irTest.duties[0] = MIN_CHARGE_DUTY_CYCLE + 10;
                        s_irTest.duties[1] = MIN_CHARGE_DUTY_CYCLE + 40;
                        s_irTest.duties[2] = MIN_CHARGE_DUTY_CYCLE + 80;
                        s_irTest.duties[3] = MAX_CHARGE_DUTY_CYCLE / 2;
                        applyDuty(s_irTest.duties[0]);

                        // Initialize transient recording for step 1
                        s_irTest.transient_count = 0;
                        s_irTest.transient_start_ms = now;
                        s_irTest.transient_unloaded_V = v;
                        s_irTest.transient_i_before = cur;
                        s_irTest.last_sample_ms = 0;
                    }
                } else if (s_irTest.step >= 1 && s_irTest.step <= 4) {
                    // Collect transient samples during step 1 for RC fitting
                    if (s_irTest.step == 1 && s_irTest.transient_count < StructuredIRTest::TRANSIENT_MAX_SAMPLES) {
                        if (s_irTest.last_sample_ms == 0 || now - s_irTest.last_sample_ms >= 10) {
                            s_irTest.transient_time_s[s_irTest.transient_count] = (now - s_irTest.transient_start_ms) * 0.001f;
                            s_irTest.transient_voltage_V[s_irTest.transient_count] = v;
                            s_irTest.transient_current_A[s_irTest.transient_count] = cur;
                            s_irTest.transient_count++;
                            s_irTest.last_sample_ms = now;
                        }
                    }

                    if (stepElapsed >= reqStepDelay) {
                        s_irTest.voltages[s_irTest.step - 1] = v;
                        s_irTest.currents[s_irTest.step - 1] = cur;

                        if (s_irTest.step == 1) {
                            // Update electrode parameters with full step response transient data
                            evaluateElectrodeParameters(
                                s_irTest.transient_time_s,
                                s_irTest.transient_voltage_V,
                                s_irTest.transient_current_A,
                                s_irTest.transient_count,
                                s_irTest.transient_unloaded_V,
                                s_irTest.transient_i_before
                            );
                        }
                        if (s_irTest.step < 4) {
                            s_irTest.step++;
                            s_irTest.stepStartTime = now;
                            applyDuty(s_irTest.duties[s_irTest.step - 1]);
                        } else {
                            // Compute structured IR from 5 sweep points (0, V_unloaded) plus the 4 loaded points
                            // using simple linear regression. This anchors the regression at zero current!
                            float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

                            // Point 0: Unloaded (0, V_unloaded)
                            sumX += 0.0f;
                            sumY += s_irTest.unloadedVoltage;
                            sumXY += 0.0f;
                            sumX2 += 0.0f;

                            // Points 1-4: Loaded
                            for (int k = 0; k < 4; k++) {
                                float I = s_irTest.currents[k];
                                float V = s_irTest.voltages[k];
                                sumX += I;
                                sumY += V;
                                sumXY += I * V;
                                sumX2 += I * I;
                            }
                            float denom = (5 * sumX2 - sumX * sumX);
                            if (std::abs(denom) > 1e-6f) {
                                float slope = (5 * sumXY - sumX * sumY) / denom;
                                s_irTest.calculatedIR = std::fabs(slope);
                            }
                            if (s_irTest.calculatedIR < MIN_VALID_RESISTANCE || s_irTest.calculatedIR > STRUCTURED_IR_SWEEP_MAX_LIMIT) {
                                s_irTest.calculatedIR = STRUCTURED_IR_SWEEP_DEFAULT_FALLBACK; // Reasonable default
                            }

                            WEB_LOCK();
                            regressedInternalResistancePairsIntercept = s_irTest.calculatedIR;
                            regressedInternalResistanceIntercept = s_irTest.calculatedIR;
                            // Inject into the original resistance arrays for UI compatibility
                            storeOrAverageResistanceData(s_irTest.currents[3], s_irTest.calculatedIR, internalResistanceDataPairs, resistanceDataCountPairs);
                            storeOrAverageResistanceData(s_irTest.currents[3], s_irTest.calculatedIR, internalResistanceData, resistanceDataCount);
                            WEB_UNLOCK();

                            updateDynamicMaximumCurrent();
                            Serial.printf("Structured IR Pulse Test Complete: IR = %.4f Ohms\n", s_irTest.calculatedIR);

                            // Check if we should execute alike original system IR re-measurement
                            selectRandomRePoints();
                            if (s_reMeasure.active) {
                                Serial.printf("Transitioning to Pulse IR Re-measurement of %d points...\n", (int)s_reMeasure.points.size());
                                chargingState = CHARGE_PULSE_IR_REMEASURE;
                                s_reMeasure.index = 0;
                                s_reMeasure.subStep = 0;
                                s_reMeasure.stepStartTime = now;

                                extern int minimalDutyCycle;
                                int minDC = minimalDutyCycle;
                                if (minDC < MIN_DUTY_CYCLE_START) {
                                    minDC = estimateDutyCycleForCurrent(MEASURABLE_CURRENT_THRESHOLD);
                                }
                                if (minDC < MIN_DUTY_CYCLE_START) {
                                    minDC = MIN_DUTY_CYCLE_START;
                                }
                                const RePoint& firstPt = s_reMeasure.points[0];
                                applyDuty(firstPt.isPair ? minDC : 0);
                            } else {
                                // Transition directly to charging pulse
                                chargingState = CHARGE_PULSE_ACTIVE;
                                pulseCycleStartTime = now;
                                pulseCurrentSum = 0.0;
                                pulseCurrentSamples = 0;
                                prev_t1 = -1.0; // Reset derivative trackers to prevent spikes across state boundary
                                prev_t2 = -1.0;
                            prev_divergence_m_set = false;
                            dD_dt_smooth = 0.0;
                            P_residual_slow = 0.0;
                                // Set constant current charging pulse duty cycle
                                int optimalDC = estimateDutyCycleForCurrent(maximumCurrent);
                                applyDuty(std::max(MIN_CHARGE_DUTY_CYCLE, std::min(MAX_CHARGE_DUTY_CYCLE, optimalDC)));
                            }
                        }
                    }
                }
            }
            break;

        case CHARGE_PULSE_IR_REMEASURE:
            {
                if (!s_reMeasure.active || s_reMeasure.index >= (int)s_reMeasure.points.size()) {
                    chargingState = CHARGE_PULSE_ACTIVE;
                    pulseCycleStartTime = now;
                    prev_t1 = -1.0;
                    prev_t2 = -1.0;
                    prev_divergence_m_set = false;
                    dD_dt_smooth = 0.0;
                    P_residual_slow = 0.0;
                    int optimalDC = estimateDutyCycleForCurrent(maximumCurrent);
                    applyDuty(std::max(MIN_CHARGE_DUTY_CYCLE, std::min(MAX_CHARGE_DUTY_CYCLE, optimalDC)));
                    break;
                }

                unsigned long stepElapsed = now - s_reMeasure.stepStartTime;
                double t1, t2, td; float tmv, v, cur; getThermistorReadings(t1, t2, td, tmv, v, cur);
                const RePoint& pt = s_reMeasure.points[s_reMeasure.index];

                ElectrodeParameters electrodeSnap = getElectrodeParametersSnapshot();
                unsigned long reqRemeasureDelay = electrodeSnap.evaluated ? electrodeSnap.adaptiveDelayMs : PULSE_IR_REMEASURE_STABILIZATION_MS;

                if (s_reMeasure.subStep == 0) {
                    // Unloaded step: wait for adaptive stabilization
                    if (stepElapsed >= reqRemeasureDelay) {
                        s_reMeasure.unloadedVoltage = v;
                        s_reMeasure.unloadedCurrent = cur;
                        s_reMeasure.subStep = 1;
                        s_reMeasure.stepStartTime = now;
                        applyDuty(pt.duty);
                    }
                } else if (s_reMeasure.subStep == 1) {
                    // Loaded step: measure and calculate IR using adaptive stabilization
                    if (stepElapsed >= reqRemeasureDelay) {
                        extern int minimalDutyCycle;
                        int lowDC = pt.isPair ? minimalDutyCycle : 0;
                        float v1 = s_reMeasure.unloadedVoltage;
                        float i1 = s_reMeasure.unloadedCurrent;
                        int dc2 = pt.duty;
                        float v2 = v;
                        float i2 = cur;

                        float calcI = 0.0f, calcIR = 0.0f;
                        bool valid = evaluateAndCorrectPairData(lowDC, dc2, v1, v2, i1, i2, calcI, calcIR);

                        if (!valid) {
                            calcIR = s_irTest.calculatedIR;
                            calcI = cur;
                        }

                        WEB_LOCK();
                        storeOrAverageResistanceData(calcI, calcIR,
                                                     pt.isPair ? internalResistanceDataPairs : internalResistanceData,
                                                     pt.isPair ? resistanceDataCountPairs : resistanceDataCount);
                        WEB_UNLOCK();

                        Serial.printf("  Re-measured point %d/%d (%s): I=%.3fA, V_unloaded=%.3fV, V_loaded=%.3fV -> IR=%.4f Ohms\n",
                                      s_reMeasure.index + 1, (int)s_reMeasure.points.size(), pt.isPair ? "PAIR" : "L/UL",
                                      cur, s_reMeasure.unloadedVoltage, v, calcIR);

                        s_reMeasure.index++;
                        s_reMeasure.subStep = 0;
                        s_reMeasure.stepStartTime = now;

                        if (s_reMeasure.index < (int)s_reMeasure.points.size()) {
                            // Apply low load or zero load for the next point
                            const RePoint& nextPt = s_reMeasure.points[s_reMeasure.index];
                            extern int minimalDutyCycle;
                            int minDC = minimalDutyCycle;
                            if (minDC < MIN_DUTY_CYCLE_START) {
                                minDC = estimateDutyCycleForCurrent(MEASURABLE_CURRENT_THRESHOLD);
                            }
                            if (minDC < MIN_DUTY_CYCLE_START) {
                                minDC = MIN_DUTY_CYCLE_START;
                            }
                            applyDuty(nextPt.isPair ? minDC : 0);
                        } else {
                            applyDuty(0);
                        }

                        if (s_reMeasure.index >= (int)s_reMeasure.points.size()) {
                            // Finished all re-measurements. Re-perform regressions to fit internal resistance slope and intercept!
                            WEB_LOCK();
                            bubbleSort(internalResistanceData, resistanceDataCount);
                            bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

                            auto compute_spacing_threshold = [](float data[][2], int n) -> float {
                                if (n < 2) return 0.05f;
                                float minX = data[0][0], maxX = data[n-1][0];
                                float avgSpacing = (maxX - minX) / std::max(1, n-1);
                                return std::max(0.02f, avgSpacing * 1.5f);
                            };

                            distribute_error(internalResistanceData, resistanceDataCount, compute_spacing_threshold(internalResistanceData, resistanceDataCount), 1.5f);
                            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, compute_spacing_threshold(internalResistanceDataPairs, resistanceDataCountPairs), 1.5f);

                            if (resistanceDataCount >= 2) performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                            if (resistanceDataCountPairs >= 2) performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                            WEB_UNLOCK();

                            updateDynamicMaximumCurrent();
                            Serial.println("Pulse IR Re-measurement Complete. Fitted new linear regression lines.");

                            s_reMeasure.points.clear();

                            chargingState = CHARGE_PULSE_ACTIVE;
                            pulseCycleStartTime = now;
                            pulseCurrentSum = 0.0;
                            pulseCurrentSamples = 0;
                            prev_t1 = -1.0;
                            prev_t2 = -1.0;
                            prev_divergence_m_set = false;
                            dD_dt_smooth = 0.0;
                            P_residual_slow = 0.0;
                            int optimalDC = estimateDutyCycleForCurrent(maximumCurrent);
                            applyDuty(std::max(MIN_CHARGE_DUTY_CYCLE, std::min(MAX_CHARGE_DUTY_CYCLE, optimalDC)));
                        }
                    }
                }
            }
            break;

        case CHARGE_PULSE_ACTIVE:
            {
                double t1, t2, td; float tmv, v, cur; getThermistorReadings(t1, t2, td, tmv, v, cur);
                unsigned long elapsedMs = now - pulseCycleStartTime;

                // Dynamic Constant-Current Closed-Loop Regulator:
                // Adjust duty cycle dynamically to compensate for changing IR and electrochemical state,
                // clamping the current precisely to maximumCurrent.
                if (cur > maximumCurrent + DYNAMIC_REGULATOR_CURRENT_MARGIN) {
                    if (dutyCycle > MIN_CHARGE_DUTY_CYCLE) {
                        applyDuty(dutyCycle - 1);
                    }
                } else if (cur < maximumCurrent - DYNAMIC_REGULATOR_CURRENT_MARGIN) {
                    if (dutyCycle < MAX_CHARGE_DUTY_CYCLE) {
                        applyDuty(dutyCycle + 1);
                    }
                }

                unsigned long dt_ms = now - lastHousekeepTime;
                if (lastHousekeepTime == 0 || dt_ms == 0 || dt_ms > 10000) {
                    dt_ms = CHARGING_HOUSEKEEP_INTERVAL; // Fallback
                }
                float dt_s = (float)dt_ms / 1000.0f;
                lastHousekeepTime = now;

                pulseCurrentSum += cur;
                pulseCurrentSamples++;

                if (!std::isfinite(t1)) t1 = 25.0;
                if (!std::isfinite(t2)) t2 = 25.0;
                if (!std::isfinite(v)) v = 1.2f;
                if (!std::isfinite(cur) || cur < 0.0f) cur = 0.0f;

                if (prev_t1 < 0) {
                    prev_t1 = t1;
                    prev_t2 = t2;
                    t1_deriv = 0.0;
                    t2_deriv = 0.0;
                    predictedTempTrack = (float)t2; // Align tracking model with actual temperature at pulse boundary
                    if (!std::isfinite(predictedTempTrack)) predictedTempTrack = 25.0f;
                    prev_divergence_m_set = false;
                    dD_dt_smooth = 0.0;
                    P_residual_slow = 0.0;
                    lastHousekeepTime = now;
                }

                // robust derivative with simple smoothing (alpha = 0.5)
                double raw_t1_deriv = (t1 - prev_t1) / dt_s;
                double raw_t2_deriv = (t2 - prev_t2) / dt_s;
                if (!std::isfinite(raw_t1_deriv)) raw_t1_deriv = 0.0;
                if (!std::isfinite(raw_t2_deriv)) raw_t2_deriv = 0.0;

                // Physical Derivative Clamping: Guard against derivative spikes from transitions or sensor anomalies
                if (raw_t1_deriv > MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_t1_deriv = MAX_TEMPERATURE_DERIVATIVE_C_PER_S;
                if (raw_t1_deriv < -MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_t1_deriv = -MAX_TEMPERATURE_DERIVATIVE_C_PER_S;
                if (raw_t2_deriv > MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_t2_deriv = MAX_TEMPERATURE_DERIVATIVE_C_PER_S;
                if (raw_t2_deriv < -MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_t2_deriv = -MAX_TEMPERATURE_DERIVATIVE_C_PER_S;

                t1_deriv = (1.0 - TEMPERATURE_DERIVATIVE_SMOOTHING_ALPHA) * t1_deriv + TEMPERATURE_DERIVATIVE_SMOOTHING_ALPHA * raw_t1_deriv;
                t2_deriv = (1.0 - TEMPERATURE_DERIVATIVE_SMOOTHING_ALPHA) * t2_deriv + TEMPERATURE_DERIVATIVE_SMOOTHING_ALPHA * raw_t2_deriv;
                if (!std::isfinite(t1_deriv)) t1_deriv = 0.0;
                if (!std::isfinite(t2_deriv)) t2_deriv = 0.0;
                prev_t1 = t1;
                prev_t2 = t2;

                // Recover true physical temperatures to compensate for sensor thermal inertia
                float t1_true = (float)(t1 + estimatedTauSHT * t1_deriv);
                float t2_true = (float)(t2 + estimatedTauTherm * t2_deriv);
                if (!std::isfinite(t1_true)) t1_true = (float)t1;
                if (!std::isfinite(t2_true)) t2_true = (float)t2;
                float td_true = t2_true - t1_true;

                // Update real-time global recovered physical values
                recoveredAmbientTemp = t1_true;
                recoveredBatteryTemp = t2_true;

                // Complex thermal loss model evaluates loss at each time step (approx 1 step / dt_ms)
                // Predict temp change using recovered true ambient temperature (t1_true)
                // Run the standard non-linear estimateTempDiff model using predictedTempTrack and persistent file-scope unapplied energy state
                float R_load = getAverageResistanceNearCurrent(cur, internalResistanceDataPairs, resistanceDataCountPairs);
                if (!std::isfinite(R_load) || R_load < 0.01f) R_load = 0.01f;
                if (R_load > 5.0f) R_load = 5.0f;
                float predictedDiff = estimateTempDiff(v, s_irTest.unloadedVoltage, cur, R_load, t1_true, now, now - dt_ms, predictedTempTrack, &g_unappliedEnergy_J);
                if (std::isfinite(predictedDiff)) {
                    predictedTempTrack = predictedDiff + t1_true;
                } else {
                    predictedTempTrack = t2_true;
                }

                // Slow first-order residual-power estimator
                // Calculate raw measured divergence using unified lag-compensated physical temperature (t2_true)
                float D_m = t2_true - predictedTempTrack;
                if (!std::isfinite(D_m)) D_m = 0.0f;
                if (!prev_divergence_m_set) {
                    prev_divergence_m = D_m;
                    prev_divergence_m_set = true;
                    dD_dt_smooth = 0.0;
                    P_residual_slow = 0.0;
                }
                float dt_s_div = dt_s;
                double raw_div_deriv = (D_m - prev_divergence_m) / dt_s_div;
                if (!std::isfinite(raw_div_deriv)) raw_div_deriv = 0.0;

                // Physical Derivative Clamping: filter outlier spikes
                if (raw_div_deriv > MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_div_deriv = MAX_TEMPERATURE_DERIVATIVE_C_PER_S;
                if (raw_div_deriv < -MAX_TEMPERATURE_DERIVATIVE_C_PER_S) raw_div_deriv = -MAX_TEMPERATURE_DERIVATIVE_C_PER_S;

                // Smooth derivative with an alpha of 0.15
                dD_dt_smooth = 0.85 * dD_dt_smooth + 0.15 * raw_div_deriv;
                if (!std::isfinite(dD_dt_smooth)) dD_dt_smooth = 0.0;
                prev_divergence_m = D_m;

                float Cth = (std::isfinite(estimatedThermalCapacitance) && estimatedThermalCapacitance >= 5.0f && estimatedThermalCapacitance <= 50.0f)
                            ? (float)estimatedThermalCapacitance
                            : DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT;
                float G = thermalConductance_W_per_K(DEFAULT_SURFACE_AREA_M2, estimatedConvectiveH, DEFAULT_EMISSIVITY, t1_true + 273.15f);
                float P_inst = Cth * dD_dt_smooth + G * D_m;
                if (!std::isfinite(P_inst)) P_inst = 0.0f;

                // Estimate P_residual slowly with a true 20s continuous time constant independent of sample rate
                float alpha_p = 1.0f - expf(-dt_s / 20.0f);
                P_residual_slow += (double)alpha_p * ((double)P_inst - P_residual_slow);
                if (!std::isfinite(P_residual_slow)) P_residual_slow = 0.0;
                float p_residual = (float)P_residual_slow;

                // Integrate unexplained thermal power using a leaky integrator (relaxation time constant = 60s)
                residualEnergy_J = std::max(0.0f, residualEnergy_J * expf(-dt_s / 60.0f) + p_residual * dt_s);
                if (!std::isfinite(residualEnergy_J)) residualEnergy_J = 0.0f;

                // Self-tune baseline during first 30 seconds of active charging pulse cycle (based on elapsed time)
                if (!baseline_calibrated) {
                    if (now - pulseCycleStartTime <= 30000) {
                        if (p_residual < 0.25f && baseline_pres_count < 120) {
                            baseline_pres_samples[baseline_pres_count++] = p_residual;
                        }
                    } else {
                        if (baseline_pres_count > 5) {
                            float sum = 0.0f;
                            for (int i = 0; i < baseline_pres_count; i++) sum += baseline_pres_samples[i];
                            baseline_mean = sum / (float)baseline_pres_count;

                            float var = 0.0f;
                            for (int i = 0; i < baseline_pres_count; i++) {
                                float diff = baseline_pres_samples[i] - baseline_mean;
                                var += diff * diff;
                            }
                            baseline_std = std::sqrt(var / (float)baseline_pres_count);

                            dynamicP_threshold = baseline_mean + 5.0f * baseline_std;
                            if (dynamicP_threshold < 0.05f) dynamicP_threshold = 0.05f;
                            if (dynamicP_threshold > 0.5f) dynamicP_threshold = 0.5f;

                            dynamicE_threshold = dynamicP_threshold * 60.0f;
                            if (dynamicE_threshold < 5.0f) dynamicE_threshold = 5.0f;
                            if (dynamicE_threshold > 30.0f) dynamicE_threshold = 30.0f;

                            baseline_calibrated = true;
                            Serial.printf("Dynamic Outgassing Thresholds Calibrated (30s elapsed): Mean=%.4fW, Std=%.4fW -> P_thresh=%.4fW, E_thresh=%.4fJ\n",
                                          baseline_mean, baseline_std, dynamicP_threshold, dynamicE_threshold);
                        } else {
                            baseline_calibrated = true;
                        }
                    }
                }

                // Electrochemical voltage prediction under load: V_predicted = V_unloaded + I * R(I) (during charging)
                float R_load_v = getAverageResistanceNearCurrent(cur, internalResistanceDataPairs, resistanceDataCountPairs);
                if (R_load_v < 0.01f) R_load_v = 0.01f;
                if (R_load_v > 5.0f) R_load_v = 5.0f;
                float predictedV = s_irTest.unloadedVoltage + cur * R_load_v;
                // Retain true signed physical overpotential direction for statistically meaningful Pearson correlation
                float overpotential = v - predictedV;

                // Expected model power dissipation (P_model = I^2 * R) normalized with a floor to prevent zero divisions
                float expectedP_model = cur * cur * R_load_v;
                float r_p_instant = p_residual / std::max(expectedP_model, 0.05f);

                // Store step response to history buffer at a sparse interval (every 5 seconds)
                // to cover a full 5 minutes (300 seconds) window with exactly 60 elements.
                static unsigned long lastThermalHistoryAppendTime = 0;
                bool appendedHistoryThisTick = false;
                WEB_LOCK();
                bool shouldAppend = s_thermalHistory.empty() || (now - lastThermalHistoryAppendTime >= THERMAL_HISTORY_LOG_INTERVAL_MS);
                WEB_UNLOCK();

                if (shouldAppend) {
                    ThermalStepResponse stepResp;
                    stepResp.timestamp = (uint32_t)now;
                    stepResp.current = cur;
                    stepResp.voltage = v;
                    stepResp.ambientTemp = t1_true;
                    stepResp.actualTemp = t2_true;
                    stepResp.predictedTemp = predictedTempTrack;
                    stepResp.predictedVoltage = predictedV;
                    stepResp.overpotential = overpotential;
                    stepResp.ir = R_load_v;
                    stepResp.p_residual = p_residual;
                    stepResp.r_p = r_p_instant;

                    WEB_LOCK();
                    s_thermalHistory.push_back(stepResp);
                    WEB_UNLOCK();

                    lastThermalHistoryAppendTime = now;
                    appendedHistoryThisTick = true;
                }

                // Detect when cell outgassing changes thermal profile (diverts from theoretical model)
                // The divergence is measured as: Actual Temp - Predicted Temp
                float divergence = t2_true - predictedTempTrack;
                float accumulatedDivergenceSum = 0.0f;
                int countDivergences = 0;

                // Track electrochemical overpotential trend to correlate with temperature rise.
                // We'll calculate the covariance or correlation between temperature divergence and overpotential over the last 5 minutes.
                // A positive correlation signifies that the electrochemically driven voltage rise is accompanied by outgassing heat divergence.
                float accumulatedOverpotentialSum = 0.0f;
                int countOverpotentials = 0;

                // Track unexplained physical residual heat power P_residual and normalized ratio r_p
                float accumulatedPresidualSum = 0.0f;
                float accumulatedRpSum = 0.0f;
                int countPresiduals = 0;

                WEB_LOCK();
                ThermalHistoryBuffer historySnap = s_thermalHistory;
                WEB_UNLOCK();

                // Look at the last 5 minutes (300 seconds) of pulse history to verify divergence and overpotential correlation
                for (size_t k = historySnap.size(); k > 0; --k) {
                    size_t idx = k - 1;
                    const auto& item = historySnap[idx];
                    if (now - item.timestamp >= 300000) break;
                    accumulatedDivergenceSum += (item.actualTemp - item.predictedTemp);
                    accumulatedOverpotentialSum += item.overpotential;
                    accumulatedPresidualSum += item.p_residual;
                    accumulatedRpSum += item.r_p;
                    countDivergences++;
                    countOverpotentials++;
                    countPresiduals++;
                }

                float avgDivergence = (countDivergences > 0) ? (accumulatedDivergenceSum / countDivergences) : 0.0f;
                float avgOverpotential = (countOverpotentials > 0) ? (accumulatedOverpotentialSum / countOverpotentials) : 0.0f;
                float avgPresidual = (countPresiduals > 0) ? (accumulatedPresidualSum / countPresiduals) : 0.0f;
                float avgRp = (countPresiduals > 0) ? (accumulatedRpSum / countPresiduals) : 0.0f;

                // Calculate Pearson-like covariance/correlation coefficient over the window with a 10s lag.
                // We correlate divergence(t) with overpotential(t - 10s) using jitter-immune timestamp-based search
                // to physically compensate for sensor thermal lag.
                float covNumerator = 0.0f;
                float varDivergence = 0.0f;
                float varOverpotential = 0.0f;

                int activeCount = 0;
                float sumDivergenceForLag = 0.0f;
                float sumLaggedOverpotential = 0.0f;

                auto get_lagged_overpotential = [&historySnap](size_t i, uint32_t target_time) -> float {
                    if (i == 0 || historySnap.empty()) return -1.0f;
                    if (historySnap[0].timestamp > target_time) return -1.0f;
                    for (size_t j = i; j > 0; --j) {
                        size_t idx = j - 1;
                        uint32_t ts_curr = historySnap[idx].timestamp;
                        if (ts_curr <= target_time) {
                            if (idx + 1 < i) {
                                uint32_t ts_next = historySnap[idx + 1].timestamp;
                                if (ts_next > ts_curr) {
                                    float frac = (float)(target_time - ts_curr) / (float)(ts_next - ts_curr);
                                    return historySnap[idx].overpotential + frac * (historySnap[idx + 1].overpotential - historySnap[idx].overpotential);
                                }
                            }
                            uint32_t diff = target_time - ts_curr;
                            return (diff <= 3000) ? historySnap[idx].overpotential : -1.0f;
                        }
                    }
                    return -1.0f;
                };

                // Pass 1: compute averages for the lagged samples in the 5-minute window
                for (size_t i = 1; i < historySnap.size(); ++i) {
                    if (now - historySnap[i].timestamp < 300000 && historySnap[i].timestamp >= 10000) {
                        float div = historySnap[i].actualTemp - historySnap[i].predictedTemp;
                        float over = get_lagged_overpotential(i, historySnap[i].timestamp - 10000);
                        if (over >= 0.0f) {
                            sumDivergenceForLag += div;
                            sumLaggedOverpotential += over;
                            activeCount++;
                        }
                    }
                }

                float avgLaggedDivergence = (activeCount > 0) ? (sumDivergenceForLag / activeCount) : 0.0f;
                float avgLaggedOverpotential = (activeCount > 0) ? (sumLaggedOverpotential / activeCount) : 0.0f;

                // Pass 2: compute covariance and variances
                for (size_t i = 1; i < historySnap.size(); ++i) {
                    if (now - historySnap[i].timestamp < 300000 && historySnap[i].timestamp >= 10000) {
                        float over = get_lagged_overpotential(i, historySnap[i].timestamp - 10000);
                        if (over >= 0.0f) {
                            float devDiv = (historySnap[i].actualTemp - historySnap[i].predictedTemp) - avgLaggedDivergence;
                            float devOver = over - avgLaggedOverpotential;
                            covNumerator += devDiv * devOver;
                            varDivergence += devDiv * devDiv;
                            varOverpotential += devOver * devOver;
                        }
                    }
                }

                float correlation = 0.0f;
                // Enforce a minimum sample count of 20 to prevent spurious statistical correlation with sparse start-of-pulse data
                if (activeCount >= 20 && varDivergence > 1e-6f && varOverpotential > 1e-6f) {
                    correlation = covNumerator / std::sqrt(varDivergence * varOverpotential);
                    // Robustness Guard: Clamp correlation to mathematical [-1.0, 1.0] domain
                    if (std::isnan(correlation)) {
                        correlation = 0.0f;
                    } else if (correlation > 1.0f) {
                        correlation = 1.0f;
                    } else if (correlation < -1.0f) {
                        correlation = -1.0f;
                    }
                }

                // We flag outgassing if we detect a persistent unexplained heat power event
                // OR an accumulated unexplained thermal energy event, AND both exhibit a positive
                // correlation, normalized power ratio divergence (>15%), and a minimum positive
                // average overpotential (polarization > 10mV) with the electrochemical rise (establishing causal link).
                bool persistentHeating = (avgPresidual > dynamicP_threshold && avgRp > 0.15f && avgDivergence > 0.3f && correlation > 0.3f && avgOverpotential > 0.01f);
                bool accumulatedEnergyEvent = (residualEnergy_J > dynamicE_threshold && avgRp > 0.15f && avgDivergence > 0.3f && correlation > 0.3f && avgOverpotential > 0.01f);
                bool outgassingDiverged = persistentHeating || accumulatedEnergyEvent;

                // Use the correlation smoothly to scale the dynamic overtemperature safety threshold limit.
                // If correlation is positive, scale the limit down towards MIN_TEMP_DIFF_THRESHOLD.
                float corrWeight = 0.0f;
                if (correlation > 0.0f) {
                    corrWeight = correlation;
                    if (corrWeight > 1.0f) corrWeight = 1.0f;
                }
                float tempDiffThreshold = MAX_TEMP_DIFF_THRESHOLD - corrWeight * (MAX_TEMP_DIFF_THRESHOLD - MIN_TEMP_DIFF_THRESHOLD);

#ifdef MOCK_TEST
                if (appendedHistoryThisTick) {
                    std::cout << "    [DEBUG PUSH] timestamp: " << now
                              << ", actualTemp: " << t2_true
                              << ", predictedTemp: " << predictedTempTrack
                              << ", overpotential: " << overpotential
                              << ", p_residual: " << p_residual
                              << ", unloadedVoltage: " << s_irTest.unloadedVoltage
                              << ", calculatedIR: " << regressedInternalResistancePairsIntercept
                              << ", cur: " << cur << std::endl;
                }
                static int dbg_cnt = 0;
                if (dbg_cnt++ % 10 == 0) {
                    std::cout << "  [CHARGE_PULSE_ACTIVE] avgDivergence: " << avgDivergence << ", avgPresidual: " << avgPresidual << ", correlation: " << correlation << ", tempDiffThreshold: " << tempDiffThreshold << ", overpotential: " << overpotential << std::endl;
                }
#endif

                WEB_LOCK();
                MAX_DIFF_TEMP = tempDiffThreshold + predictedDiff;
                WEB_UNLOCK();

                if (!outgassingDiverged) {
                    outgassing_trip_counter = 0;
                }

                outgassingTriggered = false;
                if (outgassingDiverged) {
                    if (++outgassing_trip_counter >= OUTGASSING_TRIP_THRESHOLD) {
                        outgassingTriggered = true;
                    }
                }

                // Safety and End of Charge checks: debounce both overtemperature and outgassing triggers
                if (td_true > (MAX_DIFF_TEMP)) {
                    if (++overtemp_trip_counter >= OVERTEMP_TRIP_TRESHOLD) {
                        overtemp_trip_counter = 0;
                        outgassing_trip_counter = 0;
                        chargingState = CHARGE_STOPPED;
                        Serial.printf("End of Charge detected! Outgassing diverged: %s, Avg Divergence: %.3f C\n",
                                      outgassingTriggered ? "YES" : "NO", avgDivergence);
                    }
                } else if (td_true <= (MAX_DIFF_TEMP)) {
                    overtemp_trip_counter = 0;
                }

                if (elapsedMs >= pulseLengthMs && chargingState == CHARGE_PULSE_ACTIVE) {
                    // Log historical data at the end of each charging pulse to prevent logging phase sync issues ("buzz")
                    lastLogTime = now;
                    ChargeLogData e;
                    e.timestamp = (uint32_t)now;
                    float meanPulseCurrent = (pulseCurrentSamples > 0) ? (float)(pulseCurrentSum / pulseCurrentSamples) : cur;
                    e.current = meanPulseCurrent;
                    e.voltage = v;
                    e.ambientTemperature = (float)t1;
                    e.batteryTemperature = (float)t2;
                    {
                        float R_lu = getAverageResistanceNearCurrent(meanPulseCurrent, internalResistanceData, resistanceDataCount);
                        if (R_lu < 0.01f) R_lu = 0.01f;
                        if (R_lu > 5.0f) R_lu = 5.0f;
                        e.internalResistanceLoadedUnloaded = R_lu;
                    }
                    {
                        float R_pairs = getAverageResistanceNearCurrent(meanPulseCurrent, internalResistanceDataPairs, resistanceDataCountPairs);
                        if (R_pairs < 0.01f) R_pairs = 0.01f;
                        if (R_pairs > 5.0f) R_pairs = 5.0f;
                        e.internalResistancePairs = R_pairs;
                    }
                    e.threshold = predictedDiff;
                    logChargeData(e);
                    pushRecentChargeLog(e);

                    // Transition back to IR pulse test phase for a new combined cycle
                    chargingState = CHARGE_PULSE_IR_TEST;
                    s_irTest.step = 0;
                    s_irTest.stepStartTime = now;
                    applyDuty(0);
                    Serial.println("Transitioning to new pulse cycle IR test...");
                }
            }
            break;

        case CHARGE_STOPPED: applyDuty(0); return false;
        default: break;
    }
    return true;
}

void startCharging() { if (currentAppState != APP_STATE_CHARGING) { currentAppState = APP_STATE_CHARGING; chargingState = CHARGE_IDLE; } }
void stopCharging() {
    if (currentAppState == APP_STATE_CHARGING && chargingState != CHARGE_STOPPED) {
        chargingState = CHARGE_STOPPED;
        applyDuty(0);
        WEB_LOCK();
        s_thermalHistory.clear();
        WEB_UNLOCK();
    }
}
