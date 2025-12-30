#include "definitions.h"
#include "logging.h"
#include "graphing.h"
#include "charging.h"
#include "internal_resistance.h"
#include "home_screen.h"
#include <IRremote.h>

LGFX tft;

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

// --- External symbols (kept as-is) ---
extern void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay);
extern void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
extern void prepareTemperaturePlot();
extern void plotVoltageData();
extern void plotTemperatureData();
extern void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
extern void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
extern void displayInternalResistanceGraph();
extern void drawChargePlot(bool autoscaleX, bool autoscaleY);
extern void startCharging();
extern void measureInternalResistanceStep();
extern IRState currentIRState;


// --- Application state (minimized global footprint, explicit types) ---
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;

// Shared measurement / UI globals (volatile where modified from tasks/ISRs)
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;

uint32_t dutyCycle = 0;

// Timers
unsigned long lastPlotUpdateTime = 0;
unsigned long lastDataGatherTime = 0;
unsigned long lastChargingHouseTime = 0;
unsigned long lastIRHandleTime = 0;
unsigned long displayStateChangeTime = 0;
unsigned long idleStateStartTime = 0;

// Voltage sample throttling
volatile uint32_t voltage_last_time = 0;
volatile uint32_t voltage_update_interval = 250; // ms

// PWM pin
constexpr int pwmPin = PWM_PIN;
double THERMISTOR_1_OFFSET = 0.0; // kept for clarity

// Hardware / sensor objects (kept as-is)
CurrentModel currentModel;
SHT4xSensor sht4Sensor;
HomeScreen homeScreen;
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);



// --- Build model state (non-blocking) ---
enum class BuildModelPhase { Idle = 0, Start, SetDuty, WaitMeasurement, Finish };

BuildModelPhase buildModelPhase = BuildModelPhase::Idle;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
std::vector<float> dutyCycles;
std::vector<float> currents;

// IR critical section mutex (single, reused)
static portMUX_TYPE g_ir_mux = portMUX_INITIALIZER_UNLOCKED;

// --- Utility helpers ---
static inline void applyDuty(uint32_t duty) {
    dutyCycle = std::min<uint32_t>(duty, PWM_MAX_DUTY_CYCLE);
    analogWrite(pwmPin, static_cast<int>(dutyCycle));
}

static inline void setDisplayState(DisplayState s) {
    currentDisplayState = s;
    displayStateChangeTime = millis();
}

static inline void setAppState(AppState s) {
    currentAppState = s;
    // keep display changes explicit in caller
}

// A small wrapper to safely print debug messages (so you can later route them centrally).
static inline void debugLog(const char* msg) {
#ifdef DEBUG
    Serial.println(msg);
#endif
}

// --- PWM setup ---
void setupPWM() {
    // Some platforms expect resolution/global call; keep original semantics but use consts
    analogWriteResolution(pwmPin,PWM_RESOLUTION_BITS);
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
    pinMode(pwmPin, OUTPUT);
    applyDuty(0);
}

// --- Thermistor / sensor helpers ---
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    // wait until thermistorSensor is free; yield to avoid busy-waiting
    while (thermistorSensor.isLocked()) {
        yield();
    }

    temp1 = sht4Sensor.getTemperature();
    temp2 = thermistorSensor.getTemperature2();
    tempDiff = thermistorSensor.getDifference();
    t1_millivolts = thermistorSensor.getRawMillivolts1();

    // snapshot volatile values once
    voltage = static_cast<float>(voltage_mv) / 1000.0f;
    current = static_cast<float>(current_ma) / 1000.0f;
}

// Process thermistor data and update display/plots
void processThermistorData(const MeasurementData& data, const String& measurementType) {
    printThermistorSerial(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    updateTemperatureHistory(data.temp1, data.temp2, data.tempDiff, data.voltage, data.current);
    prepareTemperaturePlot();
    plotVoltageData();
    plotTemperatureData();
    displayTemperatureLabels(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
    tft.print(measurementType);
}

// --- Non-blocking build current model ---
void buildCurrentModelStep() {
    const unsigned long now = millis();

    switch (buildModelPhase) {
        case BuildModelPhase::Idle:
            // initialize
            debugLog("Starting fresh model building.");
            dutyCycles.clear();
            currents.clear();
            dutyCycles.push_back(0.0f);
            currents.push_back(0.0f);
            buildModelDutyCycle = 1;
            buildModelPhase = BuildModelPhase::SetDuty;
            break;

        case BuildModelPhase::SetDuty:
            if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
                applyDuty(buildModelDutyCycle);
                buildModelLastStepTime = now;
                buildModelPhase = BuildModelPhase::WaitMeasurement;
            } else {
                buildModelPhase = BuildModelPhase::Finish;
            }
            break;

        case BuildModelPhase::WaitMeasurement:
            if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
                MeasurementData data;
                getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
                data.dutyCycle = buildModelDutyCycle;
                data.timestamp = millis();

                //processThermistorData(data, "Estimating Min Current");

                if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                    currents.push_back(data.current);
                } else {
                    Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", data.current, buildModelDutyCycle);
                }

                // step up and continue
                buildModelDutyCycle += 5;
                buildModelPhase = BuildModelPhase::SetDuty;
            }
            break;

        case BuildModelPhase::Finish:
            if (dutyCycles.size() < 2) {
                Serial.println("Not enough data points to build a reliable model.");
                currentModel.isModelBuilt = false;
                applyDuty(0);
                setAppState(APP_STATE_IDLE);
                buildModelPhase = BuildModelPhase::Idle;
                return;
            }

            {
                // Fit polynomial (degree 3) in monomial basis using Eigen
                const int degree = 3;
                const int numPoints = static_cast<int>(dutyCycles.size());

                Eigen::MatrixXd A(numPoints, degree + 1);
                Eigen::VectorXd b(numPoints);

                for (int i = 0; i < numPoints; ++i) {
                    for (int j = 0; j <= degree; ++j) {
                        A(i, j) = std::pow(dutyCycles[i], j);
                    }
                    b(i) = currents[i];
                }

                Eigen::VectorXd coefficients = A.householderQr().solve(b);

                // clear constant term if needed (keeps original behavior)
                if (degree >= 0) {
                    coefficients(0) = 0.0;
                }

                currentModel.coefficients = coefficients;
                currentModel.isModelBuilt = true;

                Serial.println("Current model built successfully with coefficients:");
                for (int i = 0; i <= degree; ++i) {
                    Serial.printf("Coefficient for x^%d: %.6f\n", i, coefficients(i));
                }

                applyDuty(0);
                setAppState(APP_STATE_IDLE);
                startCharging();
                buildModelPhase = BuildModelPhase::Idle;
            }
            break;
    }
}

// Estimate current using built model
float estimateCurrent(int duty) {
    if (!currentModel.isModelBuilt) {
        Serial.println("Warning: Current model has not been built yet. Returning 0.");
        return 0.0f;
    }

    const float x = static_cast<float>(duty);
    double sum = 0.0;
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        sum += currentModel.coefficients(i) * std::pow(x, i);
    }

    if (sum < MEASURABLE_CURRENT_THRESHOLD && duty > 0) {
        Serial.printf("Estimated current (%.6f A) below threshold at duty cycle %d. Inferring.\n", sum, duty);
    }

    return static_cast<float>(std::max(0.0, sum));
}

int estimateDutyCycleForCurrent(float targetCurrent) {
    if (!currentModel.isModelBuilt) {
        return 0;
    }
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

void task_readThermistor(void* parameter) {
    mAh_last_time = millis();

    while (true) {
        const uint32_t current_time = millis();

        // Ensure SHT4x is not mid-read
        while (sht4Sensor.isLocked()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        thermistorSensor.read(sht4Sensor.getTemperature());

        // Read shunt current (oversampled)
        const int task_current_numSamples = 256;
        double sumAnalogValuesCurrent = 0.0;
        for (int i = 0; i < task_current_numSamples; ++i) {
            const uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, CURRENT_SHUNT_OVERSAMPLING);
            sumAnalogValuesCurrent += analogValue;
        }
        const double voltageAcrossShunt = (sumAnalogValuesCurrent / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
        current_ma = static_cast<float>(voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE);

        // Throttled voltage sampling
        if ((current_time - voltage_last_time) > voltage_update_interval) {
            const int task_voltage_numSamples = 256;
            double sumAnalogValuesVoltage = 0.0;
            for (int i = 0; i < task_voltage_numSamples; ++i) {
                const uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN, VOLTAGE_ATTENUATION, VOLTAGE_OVERSAMPLING);
                sumAnalogValuesVoltage += analogValue;
            }
            voltage_mv = static_cast<float>((thermistorSensor.getVCC() * MAIN_VCC_RATIO) - (sumAnalogValuesVoltage / task_voltage_numSamples));
            voltage_last_time = current_time;
        }

        // mAh integration
        const uint32_t now = millis();
        const uint32_t time_elapsed_ms = now - mAh_last_time;
        const double time_elapsed_hours = static_cast<double>(time_elapsed_ms) / 3600000.0;

        double current_for_mah_calculation_ma = current_ma; // default

        if (currentModel.isModelBuilt && (current_ma / 1000.0) < MEASURABLE_CURRENT_THRESHOLD) {
            current_for_mah_calculation_ma = static_cast<double>(estimateCurrent(dutyCycle) * 1000.0);
        }

        mAh_charged += current_for_mah_calculation_ma * time_elapsed_hours;
        mAh_last_time = now;

        if (resetAh) {
            mAh_charged = 0.0;
            resetAh = false;
            Serial.println("mAh counter reset.");
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_THERMISTOR_MS));
    }
}

#ifdef DEBUG_LABELS
#include <iostream>
int testGraph() {
    // Create some dummy data for testing
    for (int i = 0; i < 100; ++i) {
        chargeLog.push_back({(unsigned long)i * 1000,
                             0.2f + 0.1f * sin(i * 0.1f),
                             1.5f + 0.2f * cos(i * 0.05f),
                             i % 256,
                             25.0f + 0.5f * i,
                             20.0f + 0.3f * i,
                             0.1f + 0.01f * i,
                             0.2f + 0.02f * i});
    }
    drawChargePlot(true, true);
    return 0;
}
#endif // DEBUG_LABELS

// --- IR handling ---
void handleIRCommand() {
    // Only handle Samsung remote from address 0x7 (existing behavior)
    if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x7) {
        Serial.print(F("Command 0x"));
        Serial.println(IrReceiver.decodedIRData.command, HEX);

        switch (IrReceiver.decodedIRData.command) {
            case RemoteKeys::KEY_PLAY:
                setAppState(APP_STATE_MEASURING_IR);
                currentIRState = IR_STATE_START;
                setDisplayState(DISPLAY_STATE_MAIN);
                break;

            case RemoteKeys::KEY_INFO:
                if (currentAppState == APP_STATE_IDLE || currentAppState == APP_STATE_CHARGING) {
                    setDisplayState(DISPLAY_STATE_IR_GRAPH);
                    displayInternalResistanceGraph();
                }
                break;

            case RemoteKeys::KEY_SOURCE:
                if (currentDisplayState != DISPLAY_STATE_CHARGE_GRAPH) {
                    setDisplayState(DISPLAY_STATE_CHARGE_GRAPH);
                    drawChargePlot(true, true);
                }
                break;

            case RemoteKeys::KEY_MENU:
                setDisplayState((currentDisplayState != DISPLAY_STATE_IDLE) ? DISPLAY_STATE_IDLE : DISPLAY_STATE_MAIN);
                tft.fillScreen(TFT_BLACK);
                updateDisplay();
                break;

            case RemoteKeys::KEY_POWER:
                resetAh = true;
                setAppState(APP_STATE_BUILDING_MODEL);
                setDisplayState(DISPLAY_STATE_MAIN);
                buildModelPhase = BuildModelPhase::Idle; // ensure fresh start
                break;

#ifdef DEBUG_LABELS
            case RemoteKeys::KEY_0: {
                testGraph();
                delay(20000); // wait 20 seconds
                tft.fillScreen(TFT_BLACK); // clear afterwards
                break;
            }
#endif // DEBUG_LABELS

            default:
                // Unhandled key
                break;
        }
        // ensure timestamp updated where appropriate
        displayStateChangeTime = millis();
    }
}

// --- Display helpers ---
void drawLabelsFromGlobals() {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(19 * 10, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.print(mAh_charged, 3);
    tft.print(" mAh");

    if (currentAppState == APP_STATE_CHARGING) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(14 * 7, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.printf("CHARGING ");
    }

    if (currentAppState == APP_STATE_MEASURING_IR) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(14 * 7, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.printf("IR CHECK ");
        tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_RED);
    }

    if (currentAppState == APP_STATE_BUILDING_MODEL) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(14 * 7, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.printf("DUTYMODEL ");
    }
}

void updateDisplay() {
    if (currentDisplayState == DISPLAY_STATE_IDLE) {
        homeScreen.render();
        displayTemperatureLabels(temp1_values[PLOT_WIDTH - 1], temp2_values[PLOT_WIDTH - 1], diff_values[PLOT_WIDTH - 1], 0,
                                 voltage_values[PLOT_WIDTH - 1], current_values[PLOT_WIDTH - 1]);
        drawLabelsFromGlobals();
    } else if (currentDisplayState == DISPLAY_STATE_MAIN) {
        prepareTemperaturePlot();
        plotVoltageData();
        plotTemperatureData();
        displayTemperatureLabels(temp1_values[PLOT_WIDTH - 1], temp2_values[PLOT_WIDTH - 1], diff_values[PLOT_WIDTH - 1], 0,
                                 voltage_values[PLOT_WIDTH - 1], current_values[PLOT_WIDTH - 1]);
        drawLabelsFromGlobals();
    }
}

// --- Initialization ---
void setup() {
    Serial.begin(115200);
    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    IrReceiver.begin(IR_RECEIVE_PIN);

    // initialize plot arrays (external arrays assumed to exist)
    for (int i = 0; i < PLOT_WIDTH; ++i) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f;
        current_values[i] = 0.0f;
    }

    sht4Sensor.begin();
    thermistorSensor.begin();
    homeScreen.begin();

    xTaskCreate(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL);

    setupPWM();

    Serial.println("Ni-Cd/Ni-MH battery charger. use samsung DVD remote to control");
    Serial.println("PLAY to measure internal resistance of battery");
    Serial.println("INFO to see internal resistance graph");
    Serial.println("POWER to start charging");
    Serial.println("SOURCE to see charge graph");
    Serial.println("HOME to see debug graph or temp/humidity graph");
}

// --- Periodic data gather ---
void gatherData() {
    double temp1 = 0.0, temp2 = 0.0, tempDiff = 0.0;
    float t1_millivolts = 0.0f, voltage = 0.0f, current = 0.0f;
    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    //printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
}

// --- Main loop ---
void loop() {
    const unsigned long now = millis();

    // --- State machine ---
    switch (currentAppState) {
        case APP_STATE_IDLE:
            // idle work (if any) goes here
            break;

        case APP_STATE_BUILDING_MODEL:
            buildCurrentModelStep();
            break;

        case APP_STATE_MEASURING_IR:
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) {
                setAppState(APP_STATE_IDLE);
            }
            break;

        case APP_STATE_CHARGING:
            if (now - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = now;
                if (!chargeBattery()) {
                    setAppState(APP_STATE_IDLE);
                }
            }
            break;
    }

    // --- Periodic actions ---
    if (now - lastDataGatherTime >= PLOT_DATA_UPDATE_INTERVAL) {
        lastDataGatherTime = now;
        gatherData();
    }

    homeScreen.gatherData();

    if (now - lastPlotUpdateTime >= PLOT_UPDATE_INTERVAL_MS) {
        lastPlotUpdateTime = now;
        updateDisplay();
    }

    // --- IR handling (throttled) ---
    if (now - lastIRHandleTime >= IR_HANDLE_INTERVAL_MS) {
        lastIRHandleTime = now;

        portENTER_CRITICAL(&g_ir_mux);
        bool is_ir_data = IrReceiver.decode();
        portEXIT_CRITICAL(&g_ir_mux);

        if (is_ir_data) {
            handleIRCommand();
            IrReceiver.resume();
        }
    }
}
