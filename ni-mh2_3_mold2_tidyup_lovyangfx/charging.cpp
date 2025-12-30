#include "charging.h"
#include "definitions.h"
#include "logging.h"

extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
//extern void processThermistorData(const MeasurementData& data, const String& measurementType);
extern float estimateCurrent(int dutyCycle);
extern AppState currentAppState;

uint32_t chargingStartTime = 0;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;
float currentRampTarget = 0.0f;

// Monitoring evaluation snapshots (persist across states)
static float eval_mAh_snapshot = 0.0f;       // mAh at start of monitor evaluation interval
static unsigned long eval_time_snapshot = 0; // ms at start of monitor evaluation interval

// Re-evaluation (binary-search) tracking for logging (does NOT affect monitor snapshots)
static bool reeval_active = false;
static float reeval_start_mAh = 0.0f;
static unsigned long reeval_start_ms = 0;

// last re-eval metrics for logging/inspection (optional)
static float lastReeval_delta_mAh = 0.0f;
static unsigned long lastReeval_duration_ms = 0;
static float lastReeval_avgCurrent_A = 0.0f;


AsyncMeasure meas;
FindOptManager findOpt;


// --- new configurable parameters for temprise absolute/blending ---
#ifndef TEMPRISE_ABS_MAX_DEPTH
#define TEMPRISE_ABS_MAX_DEPTH 8   // hard upper bound for memory
#endif

// Defaults (change at runtime if needed)
int temprise_abs_depth = 8;        // how many recent log entries to use
float temprise_balance = 0.3f;     // final = balance*relative + (1-balance)*absolute (0..1)

// recent log circular buffer (keeps last TEMPRISE_ABS_MAX_DEPTH entries)
static ChargeLogData recentChargeLogs[TEMPRISE_ABS_MAX_DEPTH];
static int recentChargeLogsCount = 0;     // number of valid entries currently stored (0..TEMPRISE_ABS_MAX_DEPTH)
static int recentChargeLogsHead = 0;      // next write position (0..TEMPRISE_ABS_MAX_DEPTH-1)


//absolute temp rise helpers

void pushRecentChargeLog(const ChargeLogData &entry) {
    recentChargeLogs[recentChargeLogsHead] = entry;
    recentChargeLogsHead = (recentChargeLogsHead + 1) % TEMPRISE_ABS_MAX_DEPTH;
    if (recentChargeLogsCount < TEMPRISE_ABS_MAX_DEPTH) {
        ++recentChargeLogsCount;
    }
    // note: when full, head overwrites the oldest element and count stays capped
}

int indexOfOldestEntry() {
    if (recentChargeLogsCount == 0) return -1;
    // oldest = head - count (mod capacity)
    int idx = recentChargeLogsHead - recentChargeLogsCount;
    if (idx < 0) idx += TEMPRISE_ABS_MAX_DEPTH;
    return idx;
}

// -------------------------------
// Replay: compute temp rise over historical data
// -------------------------------
// depth = number of last entries to replay (1..count)
// RETURNS:
//   - NAN    -> invalid (depth<=0 or no history)
//   - 0.0f   -> depth==1 (no elapsed intervals) or no net change
//   - >0.0f  -> positive absolute rise in °C
//   - <0.0f  -> negative (cooling) — possible if ambient rises faster than heating
float computeAbsoluteTempRiseFromHistory(int depth) {
    if (depth <= 0) return NAN;
    if (recentChargeLogsCount == 0) return NAN;

    // clamp depth
    int use = depth;
    if (use > recentChargeLogsCount) use = recentChargeLogsCount;

    // find chronological oldest entry among the used set
    int oldestIndex = indexOfOldestEntry();
    if (oldestIndex < 0 || oldestIndex >= TEMPRISE_ABS_MAX_DEPTH) {
    Serial.printf("Invalid oldestIndex=%d count=%d head=%d\n",
                  oldestIndex, recentChargeLogsCount, recentChargeLogsHead);
    return NAN;
    }

    if (use < recentChargeLogsCount) {
        // we only want the most recent 'use' entries -> compute their oldest index
        oldestIndex = recentChargeLogsHead - use;
        while (oldestIndex < 0) oldestIndex += TEMPRISE_ABS_MAX_DEPTH;
        oldestIndex %= TEMPRISE_ABS_MAX_DEPTH;
    }

    // Rooting point: batteryTemperature of oldest entry (absolute)
    const ChargeLogData &root = recentChargeLogs[oldestIndex];
    double T_sim = root.batteryTemperature;       // simulated absolute battery temperature
    uint32_t prev_ts = root.timestamp;
    float final_ambient = 25 ; // final replay ambient temperature for return value
    // If only one entry used -> no time to integrate -> zero rise
    if (use == 1) return 0.0f;

    // iterate chronologically oldestIndex+1 .. up to newest used
    int idx = (oldestIndex + 1) % TEMPRISE_ABS_MAX_DEPTH;
    for (int i = 1; i < use; ++i) {
        const ChargeLogData &e = recentChargeLogs[idx];
        // ensure non-decreasing timestamps to avoid unsigned underflow
        uint32_t ts = e.timestamp;
        uint32_t dt_ms = ts - prev_ts; // handles wrap correctly
        if (dt_ms == 0) {
            // minimal positive dt to avoid zero-division or no-op
          dt_ms = 1U;
          ts = prev_ts + 1U;
         }

  //      if (dt_ms == 0) { // shouldn't happen, but skip defensively
  //          prev_ts = ts;
  //          idx = (idx + 1) % TEMPRISE_ABS_MAX_DEPTH;
  //          continue;
  //      }

        // Validate physical inputs and select resistance parameters:
        float cur = e.current;
        if (!isfinite(cur) || cur < 0.0f) cur = 0.0f;

//        float Rparam = e.internalResistancePairs;
//        if (!isfinite(Rparam) || Rparam <= 1e-12f) {
//              Rparam = regressedInternalResistancePairsIntercept;
//            if (!isfinite(Rparam) || Rparam <= 1e-12f) Rparam = 0.0f;
//        }

          float Rparam = regressedInternalResistancePairsIntercept;

        float vUnderLoad = e.voltage;
        if (!isfinite(vUnderLoad)) vUnderLoad = 0.0f;

/*
        float Rlu = e.internalResistanceLoadedUnloaded;
        if (!isfinite(Rlu) || Rlu <= 1e-12f) {
            Rlu = regressedInternalResistanceIntercept;
            if (!isfinite(Rlu) || Rlu <= 1e-12f) Rlu = 0.0f;
        }
        float vNoLoad = vUnderLoad + cur * Rlu;
*/
        float vNoLoad = vUnderLoad ; // onnly Rint simulated, not total MH

        float ambient = e.ambientTemperature;
        if (!isfinite(ambient)) ambient = 25.0f;
        final_ambient = ambient ; // update final ambient temperature

        // CRUCIAL: estimateTempDiff returns theta_new = T_bat_after - ambient
        // We must pass the *absolute* battery temperature at the start of the interval,
        // i.e. T_sim (not theta).
        float theta_new = estimateTempDiff(
            vUnderLoad,
            vNoLoad,
            cur,
            Rparam,
            ambient,
            ts,        // currentTime (end of interval)
            prev_ts,   // lastChargeEvaluationTime (start)
            T_sim      // BatteryTempC at start (absolute)
        );

        // Defensive checks:
        if (!isfinite(theta_new)) {
            // skip applying this interval (treat as no change), but advance timestamps
            // Serial.println("Replay: estimateTempDiff returned NaN for interval, skipping.");
        } else {
            // convert theta_new to absolute battery temp at end of interval:
            double T_after = theta_new + ambient;

//            // clamp per-interval jump to avoid single-bad-entry explosion
            double delta = T_after - T_sim;
            const float MAX_DELTA_PER_INTERVAL = 50.0f; // °C conservative clamp
            if (delta > MAX_DELTA_PER_INTERVAL) delta = MAX_DELTA_PER_INTERVAL;
            if (delta < -MAX_DELTA_PER_INTERVAL) delta = -MAX_DELTA_PER_INTERVAL;

//            T_sim += theta_new; // advance simulated absolute battery temp
            T_sim += delta; // advance simulated absolute battery temp

        }

        prev_ts = ts;
        idx = (idx + 1) % TEMPRISE_ABS_MAX_DEPTH;
    }

    // absolute temperature rise relative to rooting battery temperature:
//    float absoluteRise = T_sim - root.batteryTemperature;
//    float absoluteRise = T_sim - root.ambientTemperature;
    float vs_ambient = T_sim - final_ambient;
    return vs_ambient;
}



// Start an MH electrode measurement: non-blocking
void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay) {
    if (meas.active()) {
        Serial.println("Measurement already running — start ignored.");
        return;
    }
    meas.reset();
    meas.testDuty = testDutyCycle;
    meas.stabilizationDelay = stabilization_delay;
    meas.unloadedDelay = unloaded_delay;
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    meas.stateStart = millis();
    meas.state = MEAS_STOPLOAD_WAIT;
    meas.resultReady = false;
    Serial.printf("Measurement started: testDC=%d, unst=%lums, stab=%lums\n", testDutyCycle, unmanagedCastUL(meas.unloadedDelay), unmanagedCastUL(meas.stabilizationDelay));
}

bool measurementStep() {
    if (meas.state == MEAS_IDLE || meas.state == MEAS_COMPLETE || meas.state == MEAS_ABORTED) return false;

    unsigned long now = millis();

    switch (meas.state) {
        case MEAS_STOPLOAD_WAIT:
        {
            dutyCycle = 0;
            analogWrite(pwmPin, dutyCycle);

            if (now - meas.stateStart >= meas.unloadedDelay) {
                meas.unloadedData = MeasurementData();
                getThermistorReadings(meas.unloadedData.temp1, meas.unloadedData.temp2, meas.unloadedData.tempDiff,
                                      meas.unloadedData.t1_millivolts, meas.unloadedData.voltage, meas.unloadedData.current);
                meas.unloadedData.dutyCycle = 0;
                meas.unloadedData.timestamp = now;
                //processThermistorData(meas.unloadedData, "MH idle (async)");

                dutyCycle = meas.testDuty;
                analogWrite(pwmPin, meas.testDuty);
                meas.stateStart = now;
                meas.state = MEAS_APPLY_LOAD;
            }
            break;
        };

        case MEAS_APPLY_LOAD:
        {
            if (now - meas.stateStart >= meas.stabilizationDelay) {
                meas.loadedData = MeasurementData();
                getThermistorReadings(meas.loadedData.temp1, meas.loadedData.temp2, meas.loadedData.tempDiff,
                                      meas.loadedData.t1_millivolts, meas.loadedData.voltage, meas.loadedData.current);
                meas.loadedData.dutyCycle = meas.testDuty;
                meas.loadedData.timestamp = now;
                //processThermistorData(meas.loadedData, "MH loaded (async)");

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
                dutyCycle = 0;
                analogWrite(pwmPin, dutyCycle);
                Serial.printf("Measurement complete: testDC=%d U=%.3f L=%.3f I=%.3f target=%.3f diff=%.3f\n",
                              meas.testDuty, meas.result.unloadedVoltage, meas.result.loadedVoltage,
                              meas.result.current, meas.result.targetVoltage, meas.result.voltageDifference);
            }
            break;
        }

        default:
            break;
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
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        Serial.println("Measurement aborted.");
    }
}

void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval) {
    findOpt = FindOptManager();
    findOpt.active = true;
    findOpt.maxDC = (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) ? MAX_CHARGE_DUTY_CYCLE : maxChargeDutyCycle;
    findOpt.lowDC = max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
    findOpt.highDC = findOpt.maxDC;
    findOpt.optimalDC = findOpt.lowDC;
    findOpt.closestVoltageDifference = 1000.0f;
    findOpt.cache.reserve(MAX_RESISTANCE_POINTS);
    findOpt.phase = FIND_INIT_HIGHDC;
    findOpt.isReevaluation = isReeval;

    startMHElectrodeMeasurement(findOpt.highDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
    Serial.printf("FindOpt async started: measuring highDC=%d\n", findOpt.highDC);
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
                Serial.printf("FindOpt init complete: Unloaded=%.3f Loaded@%d=%.3f Curr=%.3f Target=%.3f\n",
                              findOpt.initialUnloadedVoltage, dataHigh.dutyCycle, dataHigh.loadedVoltage, dataHigh.current, findOpt.targetVoltage);
            }
        }
        return true;
    }

    if (findOpt.phase == FIND_BINARY_PREPARE) {
        if (findOpt.highDC - findOpt.lowDC <= CHARGE_CURRENT_STEP * 2) {
            distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);

            if (resistanceDataCount >= 2) {
                if (performLinearRegression(internalResistanceData, resistanceDataCount,
                                            regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
            }

            if (resistanceDataCountPairs >= 2) {
                if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                            regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of paired resistance.");
            }

            if (findOpt.optimalDC < MIN_CHARGE_DUTY_CYCLE) findOpt.optimalDC = findOpt.lowDC;
            int finalDC = findOpt.optimalDC;
            startMHElectrodeMeasurement(finalDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            findOpt.phase = FIND_FINAL_WAIT;
            Serial.printf("FindOpt finishing: scheduling final measurement at %d\n", finalDC);
            return true;
        }

        int midDC = (findOpt.lowDC + findOpt.highDC) / 2;
        startMHElectrodeMeasurement(midDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = FIND_BINARY_WAIT;
        Serial.printf("FindOpt: requested mid measurement DC=%d (low=%d high=%d)\n", midDC, findOpt.lowDC, findOpt.highDC);
        return true;
    }

    if (findOpt.phase == FIND_BINARY_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData cur;
            if (fetchMeasurementResult(cur)) {
                if (cur.current < MEASURABLE_CURRENT_THRESHOLD) {
                    cur.current = estimateCurrent(cur.dutyCycle);
                }

                if (cur.current > 0.001f) { // to avoid division by 0
                    float internalResistanceLU = (cur.unloadedVoltage - cur.loadedVoltage) / cur.current;
                    storeOrAverageResistanceData(cur.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }

                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(cur);

                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(cur.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistancePair = (cached.loadedVoltage - cur.loadedVoltage) / (cur.current - cached.current);
                        float higherCurrent = std::max(cur.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

                    }
                }

                float curDiff = fabs(cur.loadedVoltage - findOpt.targetVoltage);
                Serial.printf("FindOpt measured DC=%d loaded=%.3f target=%.3f diff=%.3f curr=%.3f\n",
                              cur.dutyCycle, cur.loadedVoltage, findOpt.targetVoltage, curDiff, cur.current);

                if (cur.loadedVoltage < findOpt.targetVoltage) {
                    findOpt.lowDC = cur.dutyCycle;
                } else {
                    findOpt.highDC = cur.dutyCycle;
                }

                if (curDiff < findOpt.closestVoltageDifference) {
                    findOpt.closestVoltageDifference = curDiff;
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
                        float higherCurrent = std::max(finalData.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                }

                distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
                distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);

                if (resistanceDataCount >= 2) {
                    if (performLinearRegression(internalResistanceData, resistanceDataCount,
                                                regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                        Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                      regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                    }
                } else {
                    Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
                }

                if (resistanceDataCountPairs >= 2) {
                    if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                                regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                        Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                      regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                    }
                } else {
                    Serial.println("Not enough data points for linear regression of paired resistance.");
                }

                findOpt.targetVoltage = findOpt.initialUnloadedVoltage +
                                        (finalData.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

                Serial.printf("FindOpt complete: optimalDC=%d loaded=%.3f target=%.3f diff=%.3f\n",
                              findOpt.optimalDC, finalData.loadedVoltage, findOpt.targetVoltage,
                              fabs(finalData.loadedVoltage - findOpt.targetVoltage));

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

static float computeDissipatedPower(float vUnderLoad, float vNoLoad, float current, float Rparam) {
  const float epsI = 1e-9f;
  float P_v = 0.0f;
  if (current > epsI) {
    float vDrop = vNoLoad - vUnderLoad;
    if (vDrop > 1e-6f) {
      P_v = current * vDrop;
    }
  }
  float P_r = 0.0f;
  if (Rparam > 0.0f) {
    P_r = current * current * Rparam;
  }
  if (P_v > 0.0f && P_r > 0.0f) return 0.5f * (P_v + P_r);
  return (P_v > 0.0f) ? P_v : P_r;
}

static float thermalConductance_W_per_K(float area, float h, float emissivity, float T_ambientK) {
  return h * area + 4.0f * emissivity * STEFAN_BOLTZMANN * area * powf(T_ambientK, 3.0f);
}

float estimateTempDiff(
  float voltageUnderLoad,
  float voltageNoLoad,
  float current,
  float internalResistanceParam,
  float ambientTempC,
  uint32_t currentTime,
  uint32_t lastChargeEvaluationTime,
  float BatteryTempC,
  float cellMassKg,
  float specificHeat,
  float area,
  float convectiveH,
  float emissivity
) {
  uint32_t dt_ms = (uint32_t)(currentTime - lastChargeEvaluationTime);
  float dt_s = (float)dt_ms * 0.001f;
  if (dt_s <= 0.0f) return BatteryTempC- ambientTempC;

  float P = computeDissipatedPower(voltageUnderLoad, voltageNoLoad, current, internalResistanceParam);

  float T_amb_K = ambientTempC + 273.15f;

  float G = thermalConductance_W_per_K(area, convectiveH, emissivity, T_amb_K);

  float Cth = cellMassKg * specificHeat;

  const float tiny = 1e-12f;
  float theta0 = BatteryTempC - ambientTempC; // initial relative temperature
  if (G <= tiny) {
    float deltaT = (P * dt_s) / Cth;
    return theta0 + deltaT;
  }

  float theta_ss = P / G;
  float tau = Cth / G;
  float expo = expf(-dt_s / tau);
  float theta_new = theta_ss + (theta0 - theta_ss) * expo;
  return theta_new;
}

bool chargeBattery() {
    static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    unsigned long now = millis();

    if (chargingState != CHARGE_IDLE && chargingState != CHARGE_STOPPED && (now - chargingStartTime >= TOTAL_TIMEOUT)) {
        Serial.println("Charging timeout, stopping charging for safety reasons.");
        chargingState = CHARGE_STOPPED;
    }

    switch (chargingState) {
        case CHARGE_IDLE: {
            Serial.println("Starting battery charging process...");
            chargingStartTime = now;
            // initialize monitoring snapshots so first monitor interval is well-defined
            eval_mAh_snapshot = mAh_charged;
            eval_time_snapshot = now;

            // clear re-eval bookkeeping
            reeval_active = false;
            lastReeval_delta_mAh = 0.0f;
            lastReeval_duration_ms = 0;
            lastReeval_avgCurrent_A = 0.0f;

            lastChargeEvaluationTime = now;
            overtemp_trip_counter = 0;
            lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
            currentRampTarget = 0.0f;

            MeasurementData initialData;
            getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                                  initialData.t1_millivolts, initialData.voltage, initialData.current);
            //processThermistorData(initialData, "initial readings");
            Serial.printf("Charging started - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                          MAX_CHARGE_DUTY_CYCLE, initialData.current, initialData.temp1, initialData.temp2, initialData.tempDiff);

            ChargeLogData startLog;
            startLog.timestamp = now;
            startLog.current = initialData.current;
            startLog.voltage = initialData.voltage;
            startLog.ambientTemperature = initialData.temp1;
            startLog.batteryTemperature = initialData.temp2;
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
                // The async function has completed.
                int appliedDC = findOpt.optimalDC;
                if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
                if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;

                dutyCycle = appliedDC;
                analogWrite(pwmPin, dutyCycle);
                lastOptimalDutyCycle = dutyCycle;
                lastChargeEvaluationTime = now;

                Serial.printf("Applied optimal duty cycle: %d\n", dutyCycle);
    // --- compute re-eval deltas (for logging) ---
    lastReeval_delta_mAh = mAh_charged - reeval_start_mAh;                // mAh
    lastReeval_duration_ms = (now > reeval_start_ms) ? (now - reeval_start_ms) : 0;
    double reeval_duration_h = lastReeval_duration_ms / 3600000.0f;         // hours
    double reeval_Ah = lastReeval_delta_mAh / 1000.0f;                     // Ah
    if (reeval_duration_h > 0.0f) {
        lastReeval_avgCurrent_A = reeval_Ah / reeval_duration_h;          // A
    } else {
        lastReeval_avgCurrent_A = 0.0f;
    }

    Serial.printf("Re-eval finished: ΔmAh=%.4f mAh, dur=%lums, Iavg=%.3f A\n",
                  lastReeval_delta_mAh, lastReeval_duration_ms, lastReeval_avgCurrent_A);

    reeval_active = false;


/*  // do not log after applying optimal duty cycle as this is not final current and voltage of each charging cycle after evaluation
                MeasurementData afterApply;
                getThermistorReadings(afterApply.temp1, afterApply.temp2, afterApply.tempDiff,
                                      afterApply.t1_millivolts, afterApply.voltage, afterApply.current);
                ChargeLogData entry;
                entry.timestamp = now;
                entry.current = afterApply.current;
                entry.voltage = afterApply.voltage;
                entry.ambientTemperature = afterApply.temp1;
                entry.batteryTemperature = afterApply.temp2;
                entry.dutyCycle = dutyCycle;
                entry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
                entry.internalResistancePairs = regressedInternalResistancePairsIntercept;
                logChargeData(entry);
*/

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
                dutyCycle--;
                analogWrite(pwmPin, dutyCycle);
              }
            }

   //         if (now - lastChargeEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
              if (now - eval_time_snapshot >= CHARGE_EVALUATION_INTERVAL_MS) {
                currentRampTarget += maximumCurrent * 0.10f; // Increase by 10% of the max current
                if (currentRampTarget > maximumCurrent) {
                    currentRampTarget = maximumCurrent; // Clamp to the absolute maximum
                } else {
                  Serial.printf("Ramping up. New current target: %.3f A\n", currentRampTarget);
                }

    // compute full delta since last monitor evaluation (this INCLUDES any re-eval energy/time)
    double delta_mAh = mAh_charged - eval_mAh_snapshot;      // mAh
    unsigned long delta_ms = now - eval_time_snapshot;      // ms
    double delta_h = delta_ms / 3600000.0f;                  // hours

    double avgCurrentA = 0.0f;
    if (delta_h > 0.0f) {
        double delta_Ah = delta_mAh / 1000.0f;               // convert mAh -> Ah
        avgCurrentA = delta_Ah / delta_h;                  // amps (A)
    }

    Serial.printf("Monitor eval: ΔmAh=%.4f mAh over %lums => Iavg=%.3f A\n",
                  delta_mAh, delta_ms, avgCurrentA);


                Serial.println("end of charge by temperature delta check...");
//                float tempRise_relative = estimateTempDiff(voltage, voltage, current,
//                                                  regressedInternalResistancePairsIntercept,
//                                                  temp1, now, lastChargeEvaluationTime, temp2);
   // Use averaged current (from mAh counter) instead of instantaneous current
    float tempRise_relative = estimateTempDiff(
        voltage, voltage, avgCurrentA,
        regressedInternalResistancePairsIntercept,
        temp1, now, eval_time_snapshot, temp2
    );


                Serial.print("Estimated temperature rise due to Rint heating (relative): ");
                Serial.print(tempRise_relative);
                Serial.println(" °C");


            //    MAX_DIFF_TEMP = (MAX_TEMP_DIFF_THRESHOLD + tempRise);

// logging
                MeasurementData afterApply;
                getThermistorReadings(afterApply.temp1, afterApply.temp2, afterApply.tempDiff,
                                      afterApply.t1_millivolts, afterApply.voltage, afterApply.current);
                ChargeLogData entry;
                entry.timestamp = now;
 //               entry.current = afterApply.current;
                entry.current = avgCurrentA;

                entry.voltage = afterApply.voltage;
                entry.ambientTemperature = afterApply.temp1;
                entry.batteryTemperature = afterApply.temp2;
                entry.dutyCycle = dutyCycle;
                entry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
                entry.internalResistancePairs = regressedInternalResistancePairsIntercept;
                logChargeData(entry);
                pushRecentChargeLog(entry);

// --- compute absolute temprise by replaying history ---
    float temprise_absolute = NAN;
    if (recentChargeLogsCount >= 1) {
        // use requested depth but cap to available count
        int depthToUse = temprise_abs_depth;
        if (depthToUse > recentChargeLogsCount) depthToUse = recentChargeLogsCount;

        temprise_absolute = computeAbsoluteTempRiseFromHistory(depthToUse);

        if (!isnan(temprise_absolute)) {
            Serial.print("Estimated temperature rise (absolute, replay of last ");
            Serial.print(depthToUse);
            Serial.print(" entries): ");
            Serial.print(temprise_absolute);
            Serial.println(" °C");
        } else {
            Serial.println("computeAbsoluteTempRiseFromHistory returned NAN — not enough or invalid history.");
        }
    } else {
        Serial.println("No recent log history available to compute absolute temprise.");
    }

    // Fallback: if absolute computation failed, use relative
    if (isnan(temprise_absolute)) temprise_absolute = tempRise_relative;

    // clamp negligible negatives -> zero (optional)
    if (temprise_absolute < 0.0f && fabs(temprise_absolute) < 1e-4f) temprise_absolute = 0.0f;
    if (tempRise_relative < 0.0f && fabs(tempRise_relative) < 1e-4f) tempRise_relative = 0.0f;

    // Blend according to temprise_balance
    if (temprise_balance < 0.0f) temprise_balance = 0.0f;
    if (temprise_balance > 1.0f) temprise_balance = 1.0f;
    float finalTempRise = temprise_balance * tempRise_relative + (1.0f - temprise_balance) * temprise_absolute;

    Serial.print("Final temprise (blended, balance=");
    Serial.print(temprise_balance);
    Serial.print("): ");
    Serial.print(finalTempRise);
    Serial.println(" °C");

   eval_mAh_snapshot = mAh_charged;
    eval_time_snapshot = now;

    // also clear lastReeval_* if you want them only used once per monitor interval
    lastReeval_delta_mAh = 0.0f;
    lastReeval_duration_ms = 0;
    lastReeval_avgCurrent_A = 0.0f;

    // keep lastChargeEvaluationTime in sync for other uses
    lastChargeEvaluationTime = now;


    // apply final blended value to threshold
    MAX_DIFF_TEMP = (MAX_TEMP_DIFF_THRESHOLD + finalTempRise);

            if (currentRampTarget >= maximumCurrent) { // only check if temp threshold is exceeded when current ramping is finished and temperatures settled
                if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD + finalTempRise)) {
                    Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C).\n", tempDiff, MAX_DIFF_TEMP);
                    if (overtemp_trip_counter++ >= OVERTEMP_TRIP_TRESHOLD) {
                        overtemp_trip_counter = 0;
                        chargingState = CHARGE_STOPPED;
                        Serial.println("Over-temperature trip: charging stopped.");
                    } else {
                        Serial.printf("Over-temp transient: trip counter now %d (will re-check next evaluation).\n", overtemp_trip_counter);
                        //lastChargeEvaluationTime = now;
                    }
                }
            }

                if (chargingState == CHARGE_MONITOR) {
                    Serial.println("Re-evaluating charging parameters (non-blocking)...");
                    reeval_active = true;
                    reeval_start_mAh = mAh_charged;
                    reeval_start_ms = now; //snapshot of energy and time

 //                   int suggestedStartDutyCycle = min(lastOptimalDutyCycle + (int)(1.0 * lastOptimalDutyCycle),MAX_CHARGE_DUTY_CYCLE);
                    int suggestedStartDutyCycle = min( (int)(0.5 * lastOptimalDutyCycle),MAX_CHARGE_DUTY_CYCLE);
                    int suggestedEndDutyCycle   = max( (int)(2 * lastOptimalDutyCycle),MIN_CHARGE_DUTY_CYCLE);
                    startFindOptimalManagerAsync(suggestedEndDutyCycle, suggestedStartDutyCycle, true);
                    chargingState = CHARGE_FIND_OPT;
                }
            }
            break;
        }

        case CHARGE_STOPPED: {
            dutyCycle = 0;
            analogWrite(pwmPin, 0);
            return false; // Signal that charging is complete.
        }

        default:
            break;
    }

    return true; // Signal that charging is ongoing.
}

void startCharging() {
    if (currentAppState != APP_STATE_CHARGING) {
        currentAppState = APP_STATE_CHARGING;
        chargingState = CHARGE_IDLE;
        Serial.println("Initiating battery charging...");
        tft.setTextColor(TFT_RED,TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.printf("CHARGING");
    } else {
        Serial.println("Charging already in progress");
    }
}

void stopCharging() {
  if (currentAppState == APP_STATE_CHARGING && chargingState != CHARGE_STOPPED) {
    Serial.println("Manually stopping charging");
    chargingState = CHARGE_STOPPED;
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("STOPPED");
  }
}
