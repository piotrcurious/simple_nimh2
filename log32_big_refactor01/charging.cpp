#include "charging.h"
#include "definitions.h"
#include "logging.h"

extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
extern void processThermistorData(const MeasurementData& data, const String& measurementType);
extern float estimateCurrent(int dutyCycle);
extern AppState currentAppState;

uint32_t chargingStartTime = 0;
bool isCharging = false;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;

AsyncMeasure meas;
FindOptManager findOpt;


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
                processThermistorData(meas.unloadedData, "MH idle (async)");

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
                processThermistorData(meas.loadedData, "MH loaded (async)");

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

                if (cur.current > 0.01f) {
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

               float Rint_est = (finalData.current > 0.01f)
                                 ? fabs((finalData.unloadedVoltage - finalData.loadedVoltage) / finalData.current)
                                 : 0.0f;

                float unclampedTarget = findOpt.initialUnloadedVoltage +
                                        (finalData.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

                if (Rint_est > 0 && maximumCurrent > 0) {
                    float maxDrop = Rint_est * maximumCurrent;
                    float maxAllowedVoltage = findOpt.initialUnloadedVoltage + maxDrop;
                    findOpt.targetVoltage = constrain(unclampedTarget, findOpt.initialUnloadedVoltage, maxAllowedVoltage);
                } else {
                    findOpt.targetVoltage = unclampedTarget;
                }

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
  if (dt_s <= 0.0f) return BatteryTempC;

  float P = computeDissipatedPower(voltageUnderLoad, voltageNoLoad, current, internalResistanceParam);

  float T_amb_K = ambientTempC + 273.15f;

  float G = thermalConductance_W_per_K(area, convectiveH, emissivity, T_amb_K);

  float Cth = cellMassKg * specificHeat;

  const float tiny = 1e-12f;
  if (G <= tiny) {
    float deltaT = (P * dt_s) / Cth;
    return BatteryTempC + deltaT;
  }

  float theta_ss = P / G;
  float theta0 = BatteryTempC - ambientTempC;
  float tau = Cth / G;
  float expo = expf(-dt_s / tau);
  float theta_new = theta_ss + (theta0 - theta_ss) * expo;
  return theta_new;
}

bool chargeBattery() {
    static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    unsigned long now = millis();

    if (isCharging && (now - chargingStartTime >= TOTAL_TIMEOUT)) {
        Serial.println("Charging timeout, stopping charging for safety reasons.");
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        chargingState = CHARGE_STOPPED;
        return false;
    }

    if (!isCharging) {
        Serial.println("Starting battery charging process...");
        isCharging = true;
        chargingStartTime = now;
        lastChargeEvaluationTime = now;
        overtemp_trip_counter = 0;
        lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;

        MeasurementData initialData;
        getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                              initialData.t1_millivolts, initialData.voltage, initialData.current);
        processThermistorData(initialData, "initial readings");
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

        startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE, false);
        chargingState = CHARGE_FIND_OPT;
        return true;
    }

    measurementStep();

    if (findOptimalChargingDutyCycleStepAsync()) {
        return true;
    }

    if (chargingState == CHARGE_FIND_OPT) {
        int appliedDC = findOpt.optimalDC;

        if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
        if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;

        dutyCycle = appliedDC;
        analogWrite(pwmPin, dutyCycle);
        lastOptimalDutyCycle = dutyCycle;
        lastChargeEvaluationTime = now;
        chargingState = CHARGE_MONITOR;

        Serial.printf("Applied optimal duty cycle: %d\n", dutyCycle);

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

        return true;
    }

    if (chargingState == CHARGE_MONITOR) {
            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);
            if (current > maximumCurrent) {
              if (dutyCycle>0){
                dutyCycle--;
                analogWrite(pwmPin,dutyCycle);
              }
            }

        if (now - chargingStartTime >= TOTAL_TIMEOUT) {
            Serial.println("Charging timeout, stopping charging for safety reasons.");
            dutyCycle = 0;
            analogWrite(pwmPin, 0);
            chargingState = CHARGE_STOPPED;
            return false;
        }

        if (now - lastChargeEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
            Serial.println("end of charge by temperature delta check...");

            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);

            float tempRise = estimateTempDiff(voltage, voltage, current,
                                              regressedInternalResistancePairsIntercept,
                                              temp1, now, lastChargeEvaluationTime, temp2);
            Serial.print("Estimated temperature rise due to Rint heating: ");
            Serial.print(tempRise);
            Serial.println(" °C");

            MAX_DIFF_TEMP = (MAX_TEMP_DIFF_THRESHOLD + tempRise);

            if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD + tempRise)) {
                Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C).\n", tempDiff, MAX_TEMP_DIFF_THRESHOLD);
                if (overtemp_trip_counter++ >= OVERTEMP_TRIP_TRESHOLD) {
                    overtemp_trip_counter = 0;
                    dutyCycle = 0;
                    analogWrite(pwmPin, 0);
                    chargingState = CHARGE_STOPPED;
                    Serial.println("Over-temperature trip: charging stopped.");
                    return false;
                } else {
                    Serial.printf("Over-temp transient: trip counter now %d (will re-check next evaluation).\n", overtemp_trip_counter);
                    lastChargeEvaluationTime = now;
                }
            }

            Serial.println("Re-evaluating charging parameters (non-blocking)...");
            dutyCycle = 0;
            analogWrite(pwmPin, 0);

            int suggestedStartDutyCycle = min(lastOptimalDutyCycle + (int)(1.0 * lastOptimalDutyCycle),
                                             MAX_CHARGE_DUTY_CYCLE);

            startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle, true);
            chargingState = CHARGE_FIND_OPT;
            return true;
        }
    }

    if (chargingState == CHARGE_STOPPED) {
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        return false;
    }

    return isCharging;
}


void startCharging() {
    if (currentAppState != APP_STATE_CHARGING) {
        currentAppState = APP_STATE_CHARGING;
        isCharging = false; // Reset charging state
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
  if (isCharging) {
    Serial.println("Manually stopping charging");
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("STOPPED");
  }
}
