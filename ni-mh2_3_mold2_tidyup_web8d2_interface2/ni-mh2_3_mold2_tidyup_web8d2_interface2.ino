#include "definitions.h"
#include "logging.h"
#include "graphing.h"
#include "charging.h"
#include "internal_resistance.h"
#include "home_screen.h"
#include "adc_dma.h"
#include "SystemDataManager.h"
#include "AdvancedPolynomialFitter.hpp"
#include <WiFi.h>
#include <WebServer.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <ArduinoEigenDense.h> // needed for QR solve

// --- Constants / configuration ---

constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION_BITS) - 1;

// Replace repeated magic numbers with named constants when appropriate
constexpr TickType_t TASK_DELAY_SHT4_MS = 100;
constexpr TickType_t TASK_DELAY_THERMISTOR_MS = 50;

// --- External symbols ---
extern void measureInternalResistanceStep();
extern volatile IRState currentIRState;
extern void handleRoot();
extern void handleData();
extern void handleCommand();


// --- Application state ---
volatile AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;

// Shared measurement / UI globals
float MEASURABLE_CURRENT_THRESHOLD = 0.005f;
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;

uint32_t dutyCycle = 0;

// Timers
unsigned long lastDataGatherTime = 0;
unsigned long lastChargingHouseTime = 0;

// PWM pin
constexpr int pwmPin = PWM_PIN;
double THERMISTOR_1_OFFSET = 0.0;

// Hardware / sensor objects
CurrentModel currentModel;
SHT4xSensor sht4Sensor;
HomeScreen homeScreen;
SystemDataManager systemData(sht4Sensor, THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

// Web Server
WebServer server(80);

// --- Build model state ---
volatile BuildModelPhase buildModelPhase = BuildModelPhase::Idle;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
float calibrationSum = 0;
float calibrationMax = 0;
int calibrationCount = 0;
uint32_t lastKnownSampleCount = 0;
float noiseFloorMv = 0;
std::vector<float> dutyCycles;
std::vector<float> currents;

// --- Utility helpers ---
void applyDuty(uint32_t duty) {
    dutyCycle = std::min<uint32_t>(duty, PWM_MAX_DUTY_CYCLE);
    analogWrite(pwmPin, static_cast<int>(dutyCycle));
}

static inline void setAppState(AppState s) {
    currentAppState = s;
}

// --- PWM setup ---
void setupPWM() {
    pinMode(pwmPin, OUTPUT);
    analogWriteResolution(pwmPin, PWM_RESOLUTION_BITS);
    // Use the overloaded version for ESP32
#ifndef MOCK_TEST
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
#else
    analogWriteFrequency(PWM_FREQUENCY);
#endif
    applyDuty(0);
}

// --- Thermistor / sensor helpers ---
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    SystemData d = systemData.getData();
    temp1 = d.ambient_temp_c;
    temp2 = d.battery_temp_c;
    tempDiff = d.temp_diff_c;
    t1_millivolts = 0;
    voltage = d.battery_voltage_v;
    current = d.charge_current_a;
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
    mAh_charged = d.mah_charged;
}

// --- Non-blocking build current model ---
void buildCurrentModelStep() {
    const unsigned long now = millis();
    static BuildModelPhase lastPhase = BuildModelPhase::Finish;

    if (buildModelPhase != lastPhase) {
        Serial.printf("DEBUG: buildCurrentModelStep phase transition: %d -> %d\n", (int)lastPhase, (int)buildModelPhase);
        lastPhase = buildModelPhase;
    }

    switch (buildModelPhase) {
        case BuildModelPhase::Idle:
            dutyCycles.clear();
            currents.clear();
            applyDuty(0);
            buildModelDutyCycle = 0;
            buildModelLastStepTime = now;
            buildModelPhase = BuildModelPhase::Settle;
            Serial.println("Building Current Model: Settling (2s)...");
            Serial.flush();
            break;
        case BuildModelPhase::Settle:
            if (now - buildModelLastStepTime >= 2000) {
                calibrationSum = 0;
                calibrationMax = 0;
                calibrationCount = 0;
                SystemData d = systemData.getData();
                lastKnownSampleCount = d.current_sample_count;
                buildModelLastStepTime = now; // Reset timer for Calibrate phase
                buildModelPhase = BuildModelPhase::Calibrate;
                Serial.println("Building Current Model: Calibrating Zero Offset...");
            }
            break;
        case BuildModelPhase::Calibrate:
            {
                SystemData d = systemData.getData();
                if (d.current_sample_count != lastKnownSampleCount) {
                    lastKnownSampleCount = d.current_sample_count;
                    calibrationSum += d.current_mv;
                    if (d.current_mv > calibrationMax) calibrationMax = d.current_mv;
                    calibrationCount++;
                    Serial.printf("  Calibrating... %d/20 samples (Current: %.2f mV, count=%u)\n", calibrationCount, d.current_mv, d.current_sample_count);
                }

                if (calibrationCount >= 20) {
                    float avgOffset = calibrationSum / calibrationCount;
                    systemData.setCurrentZeroOffsetMv(avgOffset);
                    noiseFloorMv = (calibrationMax - avgOffset) * 2.0f;
                    if (noiseFloorMv < 2.5f) noiseFloorMv = 2.5f;
                    Serial.printf("Auto-calibration complete. Offset: %.2f mV, NoiseFloor: %.2f mV\n", avgOffset, noiseFloorMv);
                    buildModelDutyCycle = 1;
                    applyDuty(buildModelDutyCycle); // Start applying duty immediately
                    buildModelLastStepTime = now;
                    buildModelPhase = BuildModelPhase::DetectDeadRegion;
                    Serial.println("Building Current Model: Detecting Dead Region...");
                }

                if (now - buildModelLastStepTime > 10000) { // Increased timeout
                    Serial.printf("Auto-calibration TIMEOUT (Samples: %d) - fallback to default.\n", calibrationCount);
                    systemData.setCurrentZeroOffsetMv(CURRENT_SHUNT_PIN_ZERO_OFFSET);
                    noiseFloorMv = 5.0f;
                    buildModelDutyCycle = 1;
                    applyDuty(buildModelDutyCycle);
                    buildModelLastStepTime = now;
                    buildModelPhase = BuildModelPhase::DetectDeadRegion;
                }
            }
            break;
        case BuildModelPhase::DetectDeadRegion:
            if (now - buildModelLastStepTime >= 250) {
                SystemData d = systemData.getData();
                float currentMv = d.current_mv - systemData.getCurrentZeroOffsetMv();

                Serial.printf("  Detect: Duty %d, currentMv-Offset: %.2f mV (NoiseFloor: %.2f mV), I: %.4f A\n",
                              buildModelDutyCycle, currentMv, noiseFloorMv, d.charge_current_a);

                if (currentMv > noiseFloorMv && d.charge_current_a > 0.001f) {
                    MEASURABLE_CURRENT_THRESHOLD = d.charge_current_a;
                    dutyCycles.push_back(0.0f);
                    currents.push_back(0.0f);
                    dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                    currents.push_back(d.charge_current_a);

                    Serial.printf("Dead region ends at Duty: %d, Threshold: %.3f A\n", buildModelDutyCycle, MEASURABLE_CURRENT_THRESHOLD);
                    buildModelDutyCycle += 5;
                    buildModelLastStepTime = now; // Reset timer for SetDuty
                    buildModelPhase = BuildModelPhase::SetDuty;
                } else {
                    buildModelDutyCycle += 4; // Slightly faster increment
                    if (buildModelDutyCycle > MAX_DUTY_CYCLE) {
                        Serial.println("Error: Could not detect current above noise floor.");
                        applyDuty(0);
                        setAppState(APP_STATE_IDLE);
                        buildModelPhase = BuildModelPhase::Idle;
                    } else {
                        applyDuty(buildModelDutyCycle);
                    }
                }
                buildModelLastStepTime = now;
            }
            break;
        case BuildModelPhase::SetDuty:
            if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
                applyDuty(buildModelDutyCycle);
                buildModelLastStepTime = now;
                buildModelPhase = BuildModelPhase::WaitMeasurement;
            } else {
                buildModelLastStepTime = now; // Reset timer for Finish
                buildModelPhase = BuildModelPhase::Finish;
            }
            break;
        case BuildModelPhase::WaitMeasurement:
            if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
                SystemData d = systemData.getData();
                if (d.charge_current_a >= MEASURABLE_CURRENT_THRESHOLD) {
                    dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                    currents.push_back(d.charge_current_a);
                    Serial.printf("  Model point: Duty %d, Current %.3f A, Samples: %u\n",
                                  buildModelDutyCycle, d.charge_current_a, d.current_sample_count);
                }
                buildModelDutyCycle += 5;
                buildModelPhase = BuildModelPhase::SetDuty;
            }
            break;
        case BuildModelPhase::Finish:
            if (dutyCycles.size() >= 2) {
                const int degree = 3;
                AdvancedPolynomialFitter fitter;
                std::vector<float> coeffs = fitter.fitPolynomialLebesgue(dutyCycles, currents, degree);

                currentModel.coefficients.resize(coeffs.size());
                for (size_t i = 0; i < coeffs.size(); ++i) {
                    currentModel.coefficients(i) = coeffs[i];
                }

                if (degree >= 0) currentModel.coefficients(0) = 0.0;
                currentModel.isModelBuilt = true;
                applyDuty(0);
                setAppState(APP_STATE_IDLE);
                startCharging();
            } else {
                currentModel.isModelBuilt = false;
                applyDuty(0);
                setAppState(APP_STATE_IDLE);
            }
            buildModelPhase = BuildModelPhase::Idle;
            break;
    }
}

float estimateCurrent(int duty) {
    if (!currentModel.isModelBuilt) return 0.0f;
    const float x = static_cast<float>(duty);
    double sum = 0.0;
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        sum += currentModel.coefficients(i) * std::pow(x, i);
    }
    return static_cast<float>(std::max(0.0, sum));
}

int estimateDutyCycleForCurrent(float targetCurrent) {
    if (!currentModel.isModelBuilt) return 0;
    int bestDC = 0;
    float closestCurrentDiff = std::numeric_limits<float>::max();
    for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; ++dc) {
        float estimated = estimateCurrent(dc);
        float diff = std::abs(estimated - targetCurrent);
        if (diff < closestCurrentDiff) {
            closestCurrentDiff = diff;
            bestDC = dc;
        }
    }
    return bestDC;
}

// --- Tasks ---
void task_readSHT4x(void* parameter) {
    while (true) {
        sht4Sensor.read();
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHT4_MS));
    }
}

void task_processAdcDma(void* parameter) {
    while (true) {
        processAdcDma();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_updateSystemData(void* parameter) {
    while (true) {
        if (resetAh) {
            systemData.resetMah();
            resetAh = false;
        }
        systemData.update();
        SystemData d = systemData.getData();
        voltage_mv = d.battery_voltage_v * 1000.0f;
        current_ma = d.charge_current_a * 1000.0f;
        mAh_charged = d.mah_charged;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Initialization ---
void setup() {
    Serial.begin(115200);

    WiFi.softAP("NiMH-WebCharger", "password123");
    Serial.print("Web Interface at: http://");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/command", handleCommand);
    server.begin();

    for (int i = 0; i < PLOT_WIDTH; ++i) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f;
        current_values[i] = 0.0f;
    }

    sht4Sensor.begin();
    homeScreen.begin();
    systemData.begin();
    setupAdcDma();

    xTaskCreatePinnedToCore(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_processAdcDma, "ADC_DMA", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_updateSystemData, "SYS_DATA", 4096, NULL, 1, NULL, 1);

    setupPWM();
    Serial.println("System Ready.");
}

void gatherData() {
    double temp1 = 0.0, temp2 = 0.0, tempDiff = 0.0;
    float t1_millivolts = 0.0f, voltage = 0.0f, current = 0.0f;
    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
}

void loop() {
    const unsigned long now = millis();
    server.handleClient();

    switch (currentAppState) {
        case APP_STATE_IDLE: break;
        case APP_STATE_BUILDING_MODEL: buildCurrentModelStep(); break;
        case APP_STATE_MEASURING_IR:
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) setAppState(APP_STATE_IDLE);
            break;
        case APP_STATE_CHARGING:
            if (now - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = now;
                if (!chargeBattery()) setAppState(APP_STATE_IDLE);
            }
            break;
    }

    if (now - lastDataGatherTime >= PLOT_DATA_UPDATE_INTERVAL) {
        lastDataGatherTime = now;
        gatherData();
    }
    homeScreen.gatherData();

    // Give other tasks time to run
    vTaskDelay(1);
}
