#include "charging.h"
#include "definitions.h"
#include "logging.h"


unsigned long chargingStartTime = 0;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;
float currentRampTarget = 0.0f;

// Global thermal tracking and derivative variables
double prev_t1 = -1.0;
double prev_t2 = -1.0;
double t1_deriv = 0.0;
double t2_deriv = 0.0;
float predictedTempTrack = 25.0f;
unsigned long pulseCycleStartTime = 0;
unsigned long lastLogTime = 0;

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
        if (dt_ms == 0) { ts = prev_ts + 1; dt_ms = 1; }

        float cur = e.current;
        if (!std::isfinite(cur) || cur < 0.0f) cur = 0.0f;
        float Rparam = regressedInternalResistancePairsIntercept;
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
    findOpt = FindOptManager();
    findOpt.active = true;
    findOpt.maxDC = (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) ? MAX_CHARGE_DUTY_CYCLE : maxChargeDutyCycle;
    findOpt.lowDC = std::max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
    findOpt.highDC = findOpt.maxDC;
    findOpt.optimalDC = findOpt.lowDC;
    findOpt.cache.reserve(MAX_RESISTANCE_POINTS);
    findOpt.phase = FIND_INIT_HIGHDC;
    findOpt.isReevaluation = isReeval;
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
        int dc = (findOpt.exploratory_measurement_phase == 0) ? std::max(MIN_CHARGE_DUTY_CYCLE, findOpt.lowDC - 10) : std::min(MAX_CHARGE_DUTY_CYCLE, findOpt.highDC + 10);
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
        else { std::sort(findOpt.outliers.begin(), findOpt.outliers.end(), [](const OutlierInfo& a, const OutlierInfo& b) { return a.original_index > b.original_index; }); findOpt.phase = RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE; }
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
    remeasure = RemeasureManager(); remeasure.active = true; remeasure.targetCurrent = targetCurrent;
    int predicted = estimateDutyCycleForCurrent(targetCurrent);
    remeasure.lowDC = (uint8_t)std::max(MIN_CHARGE_DUTY_CYCLE, predicted - 20);
    remeasure.highDC = (uint8_t)std::min(MAX_CHARGE_DUTY_CYCLE, predicted + 20);
    remeasure.phase = REMEASURE_BINARY_SEARCH_PREPARE;
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

static float g_unappliedEnergy_J = 0.0f;
static float g_internalReleaseTau_s = 60.0f;

static float computeDissipatedPower(float vUnderLoad, float vNoLoad, float current, float Rparam) {
  const float epsI = 1e-9f;
  float P_v = 0.0f;
  if (current > epsI) { float vDrop = vNoLoad - vUnderLoad; if (vDrop > 1e-6f) P_v = current * vDrop; }
  float P_r = 0.0f;
  if (Rparam > 0.0f) P_r = current * current * Rparam;
  if (P_v > 0.0f && P_r > 0.0f) return 0.5f * (P_v + P_r);
  return (P_v > 0.0f) ? P_v : P_r;
}

static float thermalConductance_W_per_K(float area, float h, float emissivity, float T_ambientK) {
  return h * area + 4.0f * emissivity * STEFAN_BOLTZMANN * area * powf(T_ambientK, 3.0f);
}

float estimateTempDiff(float vL, float vN, float cur, float Rp, float ambC, uint32_t now, uint32_t last, float bC, float* uE, float mass, float spec, float area, float convH, float emiss) {
  uint32_t dt_ms = now - last;
  float dt_s = (float)dt_ms * 0.001f;
  if (dt_s <= 0.0f) return bC - ambC;
  float P = computeDissipatedPower(vL, vN, cur, Rp);
  float G = thermalConductance_W_per_K(area, convH, emiss, ambC + 273.15f);
  float Cth = mass * spec;
  float theta0 = bC - ambC;
  float E_gen = P * dt_s;
  float E_rel = 0.0f;
  if (uE && *uE > 0.0f) {
    float tau_rel = std::max(1e-12f, g_internalReleaseTau_s);
    float frac = 1.0f - expf(-dt_s / tau_rel);
    E_rel = *uE * frac; *uE -= E_rel;
  }
  float E_total = E_gen + E_rel;
  if (G <= 1e-12f || Cth <= 1e-12f) return theta0 + E_total / std::max(1e-12f, Cth);
  float theta_ss = (E_total / dt_s) / G;

  // Fit the thermal prediction model using the dynamically characterized estimatedTauThermal constant
  float tau = estimatedTauThermal;
  if (tau < 5.0f || !std::isfinite(tau)) {
      tau = Cth / G; // Fallback to theoretical if not characterized
  }
  return theta_ss + (theta0 - theta_ss) * expf(-dt_s / tau);
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
};
static StructuredIRTest s_irTest;

std::vector<ThermalStepResponse> s_thermalHistory;

// Structured IR Re-measurement Subsystem Structures
struct RePoint {
    float current;
    int duty;
    bool isPair; // true: from internalResistanceDataPairs, false: from internalResistanceData
};

struct PulseIRRemeasure {
    bool active = false;
    int index = 0;
    int subStep = 0; // 0: wait unloaded, 1: wait loaded
    unsigned long stepStartTime = 0;
    float unloadedVoltage = 0.0f;
    std::vector<RePoint> points;
};
static PulseIRRemeasure s_reMeasure;

void selectRandomRePoints() {
    s_reMeasure.points.clear();
    s_reMeasure.index = 0;
    s_reMeasure.subStep = 0;
    s_reMeasure.active = false;

    // 1. Select up to 10 points randomly from internalResistanceDataPairs
    WEB_LOCK();
    int totalPairs = resistanceDataCountPairs;
    std::vector<int> pairIndices(totalPairs);
    for (int i = 0; i < totalPairs; i++) pairIndices[i] = i;
    WEB_UNLOCK();

    // Custom platform-safe shuffle
    for (int i = 0; i < totalPairs; i++) {
        int j = i + rand() % (totalPairs - i);
        std::swap(pairIndices[i], pairIndices[j]);
    }

    int selectedPairs = std::min(10, totalPairs);
    float topCurrent = estimateCurrent(MAX_DUTY_CYCLE);

    for (int k = 0; k < selectedPairs; k++) {
        int idx = pairIndices[k];
        WEB_LOCK();
        float I = internalResistanceDataPairs[idx][0];
        WEB_UNLOCK();

        // Trim/truncate currents not reachable by re-measurement
        if (I >= MEASURABLE_CURRENT_THRESHOLD && I <= topCurrent) {
            RePoint p;
            p.current = I;
            p.duty = estimateDutyCycleForCurrent(I);
            p.isPair = true;
            s_reMeasure.points.push_back(p);
        }
    }

    // 2. Select up to 4 points randomly from internalResistanceData (loaded/unloaded)
    WEB_LOCK();
    int totalLU = resistanceDataCount;
    std::vector<int> luIndices(totalLU);
    for (int i = 0; i < totalLU; i++) luIndices[i] = i;
    WEB_UNLOCK();

    for (int i = 0; i < totalLU; i++) {
        int j = i + rand() % (totalLU - i);
        std::swap(luIndices[i], luIndices[j]);
    }

    int selectedLU = std::min(4, totalLU);
    for (int k = 0; k < selectedLU; k++) {
        int idx = luIndices[k];
        WEB_LOCK();
        float I = internalResistanceData[idx][0];
        WEB_UNLOCK();

        if (I >= MEASURABLE_CURRENT_THRESHOLD && I <= topCurrent) {
            RePoint p;
            p.current = I;
            p.duty = estimateDutyCycleForCurrent(I);
            p.isPair = false;
            s_reMeasure.points.push_back(p);
        }
    }

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
    if (pulseLengthS < 10.0f) pulseLengthS = 10.0f;
    if (pulseLengthS > 60.0f) pulseLengthS = 60.0f;
    unsigned long pulseLengthMs = (unsigned long)(pulseLengthS * 1000.0f);

    switch (chargingState) {
        case CHARGE_IDLE:
            chargingStartTime = now;
            pulseCycleStartTime = now;
            eval_mAh_snapshot = (float)mAh_charged;
            eval_time_snapshot = now;
            overtemp_trip_counter = 0;
            s_thermalHistory.clear();
            lastLogTime = 0;
            {
                double t1, t2, td; float tmv, v, c; getThermistorReadings(t1, t2, td, tmv, v, c);
                predictedTempTrack = (float)t2;
                prev_t1 = t1;
                prev_t2 = t2;
                t1_deriv = 0.0;
                t2_deriv = 0.0;
                ChargeLogData s; s.timestamp = (uint32_t)now; s.current = c; s.voltage = v; s.ambientTemperature = (float)t1; s.batteryTemperature = (float)t2;
                s.dutyCycle = 0; s.internalResistanceLoadedUnloaded = s_irTest.calculatedIR; s.internalResistancePairs = s_irTest.calculatedIR;
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

                if (s_irTest.step == 0) {
                    if (stepElapsed >= 1000) {
                        s_irTest.unloadedVoltage = v;
                        s_irTest.step = 1;
                        s_irTest.stepStartTime = now;
                        // Determine 4 duties to sweep
                        s_irTest.duties[0] = MIN_CHARGE_DUTY_CYCLE + 10;
                        s_irTest.duties[1] = MIN_CHARGE_DUTY_CYCLE + 40;
                        s_irTest.duties[2] = MIN_CHARGE_DUTY_CYCLE + 80;
                        s_irTest.duties[3] = MAX_CHARGE_DUTY_CYCLE / 2;
                        applyDuty(s_irTest.duties[0]);
                    }
                } else if (s_irTest.step >= 1 && s_irTest.step <= 4) {
                    if (stepElapsed >= 1000) {
                        s_irTest.voltages[s_irTest.step - 1] = v;
                        s_irTest.currents[s_irTest.step - 1] = cur;
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
                            if (s_irTest.calculatedIR < MIN_VALID_RESISTANCE || s_irTest.calculatedIR > 5.0f) {
                                s_irTest.calculatedIR = 0.25f; // Reasonable default
                            }

                            WEB_LOCK();
                            regressedInternalResistancePairsIntercept = s_irTest.calculatedIR;
                            regressedInternalResistanceIntercept = s_irTest.calculatedIR;
                            // Inject into the original resistance arrays for UI compatibility
                            storeOrAverageResistanceData(s_irTest.currents[3], s_irTest.calculatedIR, internalResistanceDataPairs, resistanceDataCountPairs);
                            storeOrAverageResistanceData(s_irTest.currents[3], s_irTest.calculatedIR, internalResistanceData, resistanceDataCount);
                            WEB_UNLOCK();

                            Serial.printf("Structured IR Pulse Test Complete: IR = %.4f Ohms\n", s_irTest.calculatedIR);

                            // Check if we should execute alike original system IR re-measurement
                            selectRandomRePoints();
                            if (s_reMeasure.active) {
                                Serial.printf("Transitioning to Pulse IR Re-measurement of %d points...\n", (int)s_reMeasure.points.size());
                                chargingState = CHARGE_PULSE_IR_REMEASURE;
                                s_reMeasure.index = 0;
                                s_reMeasure.subStep = 0;
                                s_reMeasure.stepStartTime = now;
                                applyDuty(0);
                            } else {
                                // Transition directly to charging pulse
                                chargingState = CHARGE_PULSE_ACTIVE;
                                pulseCycleStartTime = now;
                                prev_t1 = -1.0; // Reset derivative trackers to prevent spikes across state boundary
                                prev_t2 = -1.0;
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
                    int optimalDC = estimateDutyCycleForCurrent(maximumCurrent);
                    applyDuty(std::max(MIN_CHARGE_DUTY_CYCLE, std::min(MAX_CHARGE_DUTY_CYCLE, optimalDC)));
                    break;
                }

                unsigned long stepElapsed = now - s_reMeasure.stepStartTime;
                double t1, t2, td; float tmv, v, cur; getThermistorReadings(t1, t2, td, tmv, v, cur);
                const RePoint& pt = s_reMeasure.points[s_reMeasure.index];

                if (s_reMeasure.subStep == 0) {
                    // Unloaded step: wait for stabilization
                    if (stepElapsed >= 1000) {
                        s_reMeasure.unloadedVoltage = v;
                        s_reMeasure.subStep = 1;
                        s_reMeasure.stepStartTime = now;
                        applyDuty(pt.duty);
                    }
                } else if (s_reMeasure.subStep == 1) {
                    // Loaded step: measure and calculate IR
                    if (stepElapsed >= 1000) {
                        float measuredIR = 0.15f;
                        if (cur > 0.005f) {
                            measuredIR = (s_reMeasure.unloadedVoltage - v) / cur;
                        }
                        if (measuredIR < MIN_VALID_RESISTANCE || measuredIR > 5.0f) {
                            measuredIR = s_irTest.calculatedIR; // Fallback to sweep test if out of bounds
                        }

                        WEB_LOCK();
                        storeOrAverageResistanceData(cur, std::fabs(measuredIR),
                                                     pt.isPair ? internalResistanceDataPairs : internalResistanceData,
                                                     pt.isPair ? resistanceDataCountPairs : resistanceDataCount);
                        WEB_UNLOCK();

                        Serial.printf("  Re-measured point %d/%d (%s): I=%.3fA, V_unloaded=%.3fV, V_loaded=%.3fV -> IR=%.4f Ohms\n",
                                      s_reMeasure.index + 1, (int)s_reMeasure.points.size(), pt.isPair ? "PAIR" : "L/UL",
                                      cur, s_reMeasure.unloadedVoltage, v, measuredIR);

                        s_reMeasure.index++;
                        s_reMeasure.subStep = 0;
                        s_reMeasure.stepStartTime = now;
                        applyDuty(0);

                        if (s_reMeasure.index >= (int)s_reMeasure.points.size()) {
                            // Finished all re-measurements. Re-perform regressions to fit internal resistance slope and intercept!
                            WEB_LOCK();
                            if (resistanceDataCount >= 2) performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                            if (resistanceDataCountPairs >= 2) performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                            WEB_UNLOCK();

                            Serial.println("Pulse IR Re-measurement Complete. Fitted new linear regression lines.");

                            chargingState = CHARGE_PULSE_ACTIVE;
                            pulseCycleStartTime = now;
                            prev_t1 = -1.0;
                            prev_t2 = -1.0;
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

                if (prev_t1 < 0) {
                    prev_t1 = t1;
                    prev_t2 = t2;
                    t1_deriv = 0.0;
                    t2_deriv = 0.0;
                }

                float dt_s = (float)CHARGING_HOUSEKEEP_INTERVAL / 1000.0f;

                // robust derivative with simple smoothing (alpha = 0.5)
                double raw_t1_deriv = (t1 - prev_t1) / dt_s;
                double raw_t2_deriv = (t2 - prev_t2) / dt_s;
                t1_deriv = 0.5 * t1_deriv + 0.5 * raw_t1_deriv;
                t2_deriv = 0.5 * t2_deriv + 0.5 * raw_t2_deriv;
                prev_t1 = t1;
                prev_t2 = t2;

                // Recover true physical temperatures to compensate for sensor thermal inertia
                float t1_true = (float)(t1 + estimatedTauSHT * t1_deriv);
                float t2_true = (float)(t2 + estimatedTauTherm * t2_deriv);
                float td_true = t2_true - t1_true;

                // Complex thermal loss model evaluates loss at each time step (approx 1 step / CHARGING_HOUSEKEEP_INTERVAL)
                // Predict temp change using recovered true ambient temperature (t1_true)
                float unapplied = 0.0f;
                // Run the standard non-linear estimateTempDiff model using predictedTempTrack
                float predictedDiff = estimateTempDiff(v, s_irTest.unloadedVoltage, cur, s_irTest.calculatedIR, t1_true, now, now - CHARGING_HOUSEKEEP_INTERVAL, predictedTempTrack, &unapplied);
                predictedTempTrack = predictedDiff + t1_true;

                // Electrochemical voltage prediction: V_predicted = V_unloaded + I * R_int
                float predictedV = s_irTest.unloadedVoltage + cur * s_irTest.calculatedIR;
                float overpotential = v - predictedV;

                // Store step response
                ThermalStepResponse stepResp;
                stepResp.timestamp = (uint32_t)now;
                stepResp.current = cur;
                stepResp.voltage = v;
                stepResp.ambientTemp = t1_true;
                stepResp.actualTemp = t2_true;
                stepResp.predictedTemp = predictedTempTrack;
                stepResp.predictedVoltage = predictedV;
                stepResp.overpotential = overpotential;
                stepResp.ir = s_irTest.calculatedIR;
                s_thermalHistory.push_back(stepResp);

                // Prune/cap the history size to prevent SRAM exhaustion (only keep last 60 elements)
                if (s_thermalHistory.size() > 60) {
                    s_thermalHistory.erase(s_thermalHistory.begin(), s_thermalHistory.end() - 60);
                }

                // Detect when cell outgassing changes thermal profile (diverts from theoretical model)
                // The divergence is measured as: Actual Temp - Predicted Temp
                float divergence = (float)t2 - predictedTempTrack;
                float accumulatedDivergenceSum = 0.0f;
                int countDivergences = 0;

                // Track electrochemical overpotential trend to correlate with temperature rise.
                // We'll calculate the covariance or correlation between temperature divergence and overpotential over the last 5 minutes.
                // A positive correlation signifies that the electrochemically driven voltage rise is accompanied by outgassing heat divergence.
                float accumulatedOverpotentialSum = 0.0f;
                int countOverpotentials = 0;

                // Look at the last 5 minutes (300 seconds) of pulse history to verify divergence and overpotential correlation
                for (auto it = s_thermalHistory.rbegin(); it != s_thermalHistory.rend() && (now - it->timestamp < 300000); ++it) {
                    accumulatedDivergenceSum += (it->actualTemp - it->predictedTemp);
                    accumulatedOverpotentialSum += it->overpotential;
                    countDivergences++;
                    countOverpotentials++;
                }

                float avgDivergence = (countDivergences > 0) ? (accumulatedDivergenceSum / countDivergences) : 0.0f;
                float avgOverpotential = (countOverpotentials > 0) ? (accumulatedOverpotentialSum / countOverpotentials) : 0.0f;

                // Calculate Pearson-like covariance/correlation coefficient over the window
                float covNumerator = 0.0f;
                float varDivergence = 0.0f;
                float varOverpotential = 0.0f;
                for (auto it = s_thermalHistory.rbegin(); it != s_thermalHistory.rend() && (now - it->timestamp < 300000); ++it) {
                    float devDiv = (it->actualTemp - it->predictedTemp) - avgDivergence;
                    float devOver = it->overpotential - avgOverpotential;
                    covNumerator += devDiv * devOver;
                    varDivergence += devDiv * devDiv;
                    varOverpotential += devOver * devOver;
                }

                float correlation = 0.0f;
                if (varDivergence > 1e-6f && varOverpotential > 1e-6f) {
                    correlation = covNumerator / std::sqrt(varDivergence * varOverpotential);
                }

                // If temperature divergence exceeds threshold AND is highly correlated with the rising electrochemical overpotential (correlation > 0.6), flag outgassing.
                // This acts as a robust confirmation step.
                bool outgassingDiverged = (avgDivergence > 0.4f) && (correlation > 0.6f);
#ifdef MOCK_TEST
                std::cout << "    [DEBUG PUSH] timestamp: " << stepResp.timestamp
                          << ", actualTemp: " << stepResp.actualTemp
                          << ", predictedTemp: " << stepResp.predictedTemp
                          << ", overpotential: " << stepResp.overpotential
                          << ", unloadedVoltage: " << s_irTest.unloadedVoltage
                          << ", calculatedIR: " << s_irTest.calculatedIR
                          << ", cur: " << cur << std::endl;
                static int dbg_cnt = 0;
                if (dbg_cnt++ % 10 == 0) {
                    std::cout << "  [CHARGE_PULSE_ACTIVE] avgDivergence: " << avgDivergence << ", correlation: " << correlation << ", overpotential: " << overpotential << std::endl;
                }
#endif

                // Log historical data at robust sparse intervals (HIST_LOG_INTERVAL_MS) to prevent memory crash
                if (lastLogTime == 0 || now - lastLogTime >= HIST_LOG_INTERVAL_MS) {
                    lastLogTime = now;
                    ChargeLogData e;
                    e.timestamp = (uint32_t)now;
                    e.current = cur;
                    e.voltage = v;
                    e.ambientTemperature = (float)t1;
                    e.batteryTemperature = (float)t2;
                    e.dutyCycle = (uint8_t)dutyCycle;
                    e.internalResistanceLoadedUnloaded = s_irTest.calculatedIR;
                    e.internalResistancePairs = s_irTest.calculatedIR;
                    logChargeData(e);
                    pushRecentChargeLog(e);
                }

                WEB_LOCK();
                MAX_DIFF_TEMP = MAX_TEMP_DIFF_THRESHOLD + predictedDiff;
                WEB_UNLOCK();

                // Safety and End of Charge checks
                if (td_true > (MAX_DIFF_TEMP + 1.5f) || outgassingDiverged) {
                    if (++overtemp_trip_counter >= OVERTEMP_TRIP_TRESHOLD || outgassingDiverged) {
                        overtemp_trip_counter = 0;
                        chargingState = CHARGE_STOPPED;
                        Serial.printf("End of Charge detected! Outgassing diverged: %s, Avg Divergence: %.3f C\n",
                                      outgassingDiverged ? "YES" : "NO", avgDivergence);
                    }
                }

                if (elapsedMs >= pulseLengthMs && chargingState == CHARGE_PULSE_ACTIVE) {
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
void stopCharging() { if (currentAppState == APP_STATE_CHARGING && chargingState != CHARGE_STOPPED) { chargingState = CHARGE_STOPPED; applyDuty(0); } }
