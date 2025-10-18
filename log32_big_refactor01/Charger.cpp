#include "Charger.h"
#include "DataPlotter.h"
#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
#include "logging.h"
#include <Arduino.h>

extern SHT4xSensor sht4Sensor;
extern ThermistorSensor thermistorSensor;
extern volatile float voltage_mv;
extern volatile float current_ma;
extern AppState currentAppState;
extern std::vector<ChargeLogData> chargeLog;


Charger::Charger(int pwmPin, DataPlotter& plotter)
    : _pwmPin(pwmPin),
      _plotter(plotter),
      _dutyCycle(0),
      _buildModelStep(0),
      _buildModelDutyCycle(0),
      _buildModelLastStepTime(0),
      _lastChargingHouseTime(0)
{
}

void Charger::begin() {
    setupPWM();
    _currentModel.isModelBuilt = false;
}

void Charger::setupPWM() {
    analogWriteResolution(_pwmPin, 8);
    analogWriteFrequency(_pwmPin, PWM_FREQUENCY);
    pinMode(_pwmPin, OUTPUT);
    _dutyCycle = 0;
    analogWrite(_pwmPin, 0);
}

void Charger::startCharging() {
    if (_currentModel.isModelBuilt) {
        currentAppState = APP_STATE_CHARGING;
        Serial.println("Starting charging process.");
    } else {
        Serial.println("Cannot start charging, current model is not built yet.");
    }
}

void Charger::startModelBuilding() {
    currentAppState = APP_STATE_BUILDING_MODEL;
    _buildModelStep = 0; // Reset the state machine
    Serial.println("Starting current model building process.");
}

bool Charger::isModelBuilt() const {
    return _currentModel.isModelBuilt;
}

uint32_t Charger::getDutyCycle() const {
    return _dutyCycle;
}

void Charger::update() {
    unsigned long now = millis();
    switch (currentAppState) {
        case APP_STATE_BUILDING_MODEL:
            buildCurrentModelStep();
            break;
        case APP_STATE_CHARGING:
            if (now - _lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                _lastChargingHouseTime = now;
                if (!chargeBattery()) {
                    currentAppState = APP_STATE_IDLE;
                }
            }
            break;
        default:
            // Do nothing
            break;
    }
}

void Charger::getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    while (thermistorSensor.isLocked()) {
        yield();
    }
    temp1 = sht4Sensor.getTemperature();
    temp2 = thermistorSensor.getTemperature2();
    tempDiff = thermistorSensor.getDifference();
    t1_millivolts = thermistorSensor.getRawMillivolts1();
    voltage = voltage_mv / 1000.0f;
    current = current_ma / 1000.0f;
}

void Charger::processThermistorData(const MeasurementData& data, const String& measurementType) {
    _plotter.printThermistorSerial(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    _plotter.updateTemperatureHistory(data.temp1, data.temp2, data.tempDiff, data.voltage, data.current);
    _plotter.prepareTemperaturePlot();
    _plotter.plotVoltageData();
    _plotter.plotTemperatureData();
    _plotter.displayTemperatureLabels(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current, thermistorSensor, data.dutyCycle);
    _plotter.bigUglyMessage(measurementType);
}

void Charger::buildCurrentModelStep() {
    unsigned long now = millis();

    if (_buildModelStep == 0) {
        Serial.println("Starting fresh model building.");
        _dutyCycles.clear();
        _currents.clear();
        _dutyCycles.push_back(0.0f);
        _currents.push_back(0.0f);
        _buildModelDutyCycle = 1;
        _buildModelStep = 1;
    }

    if (_buildModelStep == 1) {
        if (_buildModelDutyCycle <= MAX_DUTY_CYCLE) {
            _dutyCycle = _buildModelDutyCycle;
            analogWrite(_pwmPin, _dutyCycle);
            _buildModelLastStepTime = now;
            _buildModelStep = 2;
        } else {
            _buildModelStep = 3;
        }
    }

    if (_buildModelStep == 2) {
        if (now - _buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
            MeasurementData data;
            getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
            data.dutyCycle = _buildModelDutyCycle;
            data.timestamp = millis();

            processThermistorData(data, "Estimating Min Current");
            if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
                _dutyCycles.push_back(static_cast<float>(_buildModelDutyCycle));
                _currents.push_back(data.current);
            } else {
                Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", data.current, _buildModelDutyCycle);
            }
            _buildModelDutyCycle += 5;
            _buildModelStep = 1;
        }
    }

    if (_buildModelStep == 3) {
        if (_dutyCycles.size() < 2) {
            Serial.println("Not enough data points to build a reliable model.");
            _currentModel.isModelBuilt = false;
            _dutyCycle = 0;
            analogWrite(_pwmPin, _dutyCycle);
            currentAppState = APP_STATE_IDLE;
            return;
        }

        int degree = 3;
        int numPoints = _dutyCycles.size();
        Eigen::MatrixXd A(numPoints, degree + 1);
        Eigen::VectorXd b(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            for (int j = 0; j <= degree; ++j) {
                A(i, j) = std::pow(_dutyCycles[i], j);
            }
            b(i) = _currents[i];
        }

        Eigen::VectorXd coefficients = A.householderQr().solve(b);
        if (degree >= 0) {
            coefficients(0) = 0.0f;
        }
        _currentModel.coefficients = coefficients;
        _currentModel.isModelBuilt = true;

        Serial.println("Current model built successfully with coefficients:");
        for (int i = 0; i <= degree; ++i) {
            Serial.printf("Coefficient for x^%d: %.4f\n", i, coefficients(i));
        }
        _dutyCycle = 0;
        analogWrite(_pwmPin, _dutyCycle);
        currentAppState = APP_STATE_IDLE;
        startCharging();
        _buildModelStep = 0;
    }
}

float Charger::estimateCurrent(int dutyCycle) {
    if (!_currentModel.isModelBuilt) {
        Serial.println("Warning: Current model has not been built yet. Returning 0.");
        return 0.0f;
    }

    float estimatedCurrent = 0.0f;
    float dutyCycleFloat = static_cast<float>(dutyCycle);
    for (int i = 0; i < _currentModel.coefficients.size(); ++i) {
        estimatedCurrent += _currentModel.coefficients(i) * std::pow(dutyCycleFloat, i);
    }

    if (estimatedCurrent < MEASURABLE_CURRENT_THRESHOLD && dutyCycle > 0) {
        // This is more of a debug message, but can be useful.
        // Serial.printf("Estimated current (%.3f A) below threshold at duty cycle %d. Inferring.\n", estimatedCurrent, dutyCycle);
    }

    return std::max(0.0f, estimatedCurrent);
}

extern ChargingState chargingState;
extern uint32_t chargingStartTime;
extern unsigned long lastChargeEvaluationTime;
extern uint8_t overtemp_trip_counter;
extern float currentRampTarget;
extern float maximumCurrent;

bool Charger::chargeBattery() {
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
            lastChargeEvaluationTime = now;
            overtemp_trip_counter = 0;
            lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
            currentRampTarget = 0.0f;

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
            extern float regressedInternalResistanceIntercept;
            extern float regressedInternalResistancePairsIntercept;
            startLog.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
            startLog.internalResistancePairs = regressedInternalResistancePairsIntercept;
            logChargeData(startLog);
            extern void pushRecentChargeLog(const ChargeLogData &entry);
            pushRecentChargeLog(startLog);

            extern void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval);
            startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE, false);
            chargingState = CHARGE_FIND_OPT;
            break;
        }
        case CHARGE_FIND_OPT: {
            extern bool findOptimalChargingDutyCycleStepAsync();
            if (!findOptimalChargingDutyCycleStepAsync()) {
                extern FindOptManager findOpt;
                int appliedDC = findOpt.optimalDC;
                if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
                if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;

                _dutyCycle = appliedDC;
                analogWrite(_pwmPin, _dutyCycle);
                lastOptimalDutyCycle = _dutyCycle;
                lastChargeEvaluationTime = now;

                Serial.printf("Applied optimal duty cycle: %d\n", _dutyCycle);

                chargingState = CHARGE_MONITOR;
            }
            break;
        }

        case CHARGE_MONITOR: {
            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);
            if (current > currentRampTarget) {
              if (_dutyCycle > 0){
                _dutyCycle--;
                analogWrite(_pwmPin, _dutyCycle);
              }
            }

            if (now - lastChargeEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
                currentRampTarget += maximumCurrent * 0.10f;
                if (currentRampTarget > maximumCurrent) {
                    currentRampTarget = maximumCurrent;
                } else {
                  Serial.printf("Ramping up. New current target: %.3f A\n", currentRampTarget);
                }

                extern float estimateTempDiff(float, float, float, float, float, uint32_t, uint32_t, float, float, float, float, float, float);
                extern float regressedInternalResistancePairsIntercept;
                float tempRise_relative = estimateTempDiff(voltage, voltage, current,
                                                  regressedInternalResistancePairsIntercept,
                                                  temp1, now, lastChargeEvaluationTime, temp2, DEFAULT_CELL_MASS_KG, DEFAULT_SPECIFIC_HEAT, DEFAULT_SURFACE_AREA_M2, DEFAULT_CONVECTIVE_H, DEFAULT_EMISSIVITY);
                Serial.print("Estimated temperature rise due to Rint heating (relative): ");
                Serial.print(tempRise_relative);
                Serial.println(" °C");

                MeasurementData afterApply;
                getThermistorReadings(afterApply.temp1, afterApply.temp2, afterApply.tempDiff,
                                      afterApply.t1_millivolts, afterApply.voltage, afterApply.current);
                ChargeLogData entry;
                entry.timestamp = now;
                entry.current = afterApply.current;
                entry.voltage = afterApply.voltage;
                entry.ambientTemperature = afterApply.temp1;
                entry.batteryTemperature = afterApply.temp2;
                entry.dutyCycle = _dutyCycle;
                extern float regressedInternalResistanceIntercept;
                entry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
                entry.internalResistancePairs = regressedInternalResistancePairsIntercept;
                logChargeData(entry);
                extern void pushRecentChargeLog(const ChargeLogData &entry);
                pushRecentChargeLog(entry);

                extern int temprise_abs_depth;
                extern int recentChargeLogsCount;
                extern float computeAbsoluteTempRiseFromHistory(int);
                float temprise_absolute = NAN;
                if (recentChargeLogsCount >= 1) {
                    int depthToUse = temprise_abs_depth;
                    if (depthToUse > recentChargeLogsCount) depthToUse = recentChargeLogsCount;
                    temprise_absolute = computeAbsoluteTempRiseFromHistory(depthToUse);
                }

                extern float temprise_balance;
                float finalTempRise = temprise_balance * tempRise_relative + (1.0f - temprise_balance) * temprise_absolute;

                if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD + finalTempRise)) {
                    Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C).\n", tempDiff, (MAX_TEMP_DIFF_THRESHOLD + finalTempRise));
                    if (overtemp_trip_counter++ >= OVERTEMP_TRIP_TRESHOLD) {
                        overtemp_trip_counter = 0;
                        chargingState = CHARGE_STOPPED;
                        Serial.println("Over-temperature trip: charging stopped.");
                    } else {
                        Serial.printf("Over-temp transient: trip counter now %d (will re-check next evaluation).\n", overtemp_trip_counter);
                        lastChargeEvaluationTime = now;
                    }
                }

                if (chargingState == CHARGE_MONITOR) {
                    Serial.println("Re-evaluating charging parameters (non-blocking)...");
                    int suggestedStartDutyCycle = min( (int)(0.5 * lastOptimalDutyCycle),MAX_CHARGE_DUTY_CYCLE);
                    int suggestedEndDutyCycle   = max( (int)(2 * lastOptimalDutyCycle),MIN_CHARGE_DUTY_CYCLE);
                    extern void startFindOptimalManagerAsync(int, int, bool);
                    startFindOptimalManagerAsync(suggestedEndDutyCycle, suggestedStartDutyCycle, true);
                    chargingState = CHARGE_FIND_OPT;
                }
            }
            break;
        }

        case CHARGE_STOPPED: {
            _dutyCycle = 0;
            analogWrite(_pwmPin, 0);
            return false;
        }

        default:
            break;
    }

    return true;
}