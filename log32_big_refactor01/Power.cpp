#include "Power.h"
#include "internal_resistance.h"
#include "DataStore.h"

Power::Power() :
    buildModelStep(0),
    buildModelDutyCycle(0),
    buildModelLastStepTime(0),
    chargingState(CHARGE_IDLE),
    chargingStartTime(0),
    overtemp_trip_counter(0),
    lastChargeEvaluationTime(0),
    maximumCurrent(0.150),
    currentRampTarget(0.0f),
    lastOptimalDutyCycle(MAX_CHARGE_DUTY_CYCLE)
{}

void Power::begin() {
    pinMode(PWM_PIN, OUTPUT);
    setDutyCycle(0);
}

void Power::setDutyCycle(uint32_t duty) {
    dutyCycle = duty;
    analogWrite(PWM_PIN, dutyCycle);
}

void Power::buildCurrentModel(DataStore& data) {
    unsigned long now = millis();

    if (buildModelStep == 0) {
        Serial.println("Starting fresh model building.");
        dutyCycles.clear();
        currents.clear();
        dutyCycles.push_back(0.0f);
        currents.push_back(0.0f);
        buildModelDutyCycle = 1;
        buildModelStep = 1;
    }

    if (buildModelStep == 1) {
        if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
            setDutyCycle(buildModelDutyCycle);
            buildModelLastStepTime = now;
            buildModelStep = 2;
        } else {
            buildModelStep = 3;
        }
    }

    if (buildModelStep == 2) {
        if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
            MeasurementData meas_data;
            meas_data.voltage = data.voltage_values[PLOT_WIDTH - 1];
            meas_data.current = data.current_values[PLOT_WIDTH - 1];
            meas_data.dutyCycle = buildModelDutyCycle;
            meas_data.timestamp = millis();

            if (meas_data.current >= MEASURABLE_CURRENT_THRESHOLD) {
                dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                currents.push_back(meas_data.current);
            } else {
                Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", meas_data.current, buildModelDutyCycle);
            }
            buildModelDutyCycle += 5;
            buildModelStep = 1;
        }
    }

    if (buildModelStep == 3) {
        if (dutyCycles.size() < 2) {
            Serial.println("Not enough data points to build a reliable model.");
            currentModel.isModelBuilt = false;
            setDutyCycle(0);
            data.currentAppState = APP_STATE_IDLE;
            return;
        }

        int degree = 3;
        int numPoints = dutyCycles.size();
        Eigen::MatrixXd A(numPoints, degree + 1);
        Eigen::VectorXd b(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            for (int j = 0; j <= degree; ++j) {
                A(i, j) = std::pow(dutyCycles[i], j);
            }
            b(i) = currents[i];
        }

        Eigen::VectorXd coefficients = A.householderQr().solve(b);
        if (degree >= 0) {
            coefficients(0) = 0.0f;
        }
        currentModel.coefficients = coefficients;
        currentModel.isModelBuilt = true;

        Serial.println("Current model built successfully with coefficients:");
        for (int i = 0; i <= degree; ++i) {
            Serial.printf("Coefficient for x^%d: %.4f\n", i, coefficients(i));
        }
        setDutyCycle(0);
        data.currentAppState = APP_STATE_IDLE;
        startCharging(data);
        buildModelStep = 0;
    }
}

bool Power::chargeBattery(DataStore& data) {
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
            currentIRState = IR_STATE_START;
            chargingState = CHARGE_FIND_OPT;
            break;
        }
        case CHARGE_FIND_OPT: {
            measureInternalResistanceStep(data);
            if (currentIRState == IR_STATE_IDLE) {
                int appliedDC = dutyCycle;
                if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
                if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;

                setDutyCycle(appliedDC);
                lastOptimalDutyCycle = dutyCycle;
                lastChargeEvaluationTime = now;

                Serial.printf("Applied optimal duty cycle: %d\n", dutyCycle);

                chargingState = CHARGE_MONITOR;
            }
            break;
        }

        case CHARGE_MONITOR: {
            if ((data.voltage_values[PLOT_WIDTH - 1]) > currentRampTarget) {
              if (dutyCycle > 0){
                setDutyCycle(dutyCycle - 1);
              }
            }

            if (now - lastChargeEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
                currentRampTarget += maximumCurrent * 0.10f;
                if (currentRampTarget > maximumCurrent) {
                    currentRampTarget = maximumCurrent;
                } else {
                  Serial.printf("Ramping up. New current target: %.3f A\n", currentRampTarget);
                }

                Serial.println("end of charge by temperature delta check...");

                if (chargingState == CHARGE_MONITOR) {
                    Serial.println("Re-evaluating charging parameters (non-blocking)...");
                    int suggestedStartDutyCycle = min( (int)(0.5 * lastOptimalDutyCycle),MAX_CHARGE_DUTY_CYCLE);
                    int suggestedEndDutyCycle   = max( (int)(2 * lastOptimalDutyCycle),MIN_CHARGE_DUTY_CYCLE);
                    currentIRState = IR_STATE_START;
                    chargingState = CHARGE_FIND_OPT;
                }
            }
            break;
        }

        case CHARGE_STOPPED: {
            setDutyCycle(0);
            return false; // Signal that charging is complete.
        }

        default:
            break;
    }

    return true; // Signal that charging is ongoing.
}

void Power::startCharging(DataStore& data) {
    if (data.currentAppState != APP_STATE_CHARGING) {
        data.currentAppState = APP_STATE_CHARGING;
        chargingState = CHARGE_IDLE;
        Serial.println("Initiating battery charging...");
    } else {
        Serial.println("Charging already in progress");
    }
}

void Power::stopCharging(DataStore& data) {
  if (chargingState != CHARGE_STOPPED) {
    Serial.println("Manually stopping charging");
    chargingState = CHARGE_STOPPED;
    setDutyCycle(0);
  }
}