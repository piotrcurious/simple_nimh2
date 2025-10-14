#include "definitions.h"
#include "logging.h"
#include "graphing.h"
#include "charging.h"
#include "internal_resistance.h"
#include <IRremote.h>

#define PLOT_WIDTH 320
#define PLOT_Y_START 0
#define PLOT_HEIGHT (216 - 3)
extern void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay);

// --- State Machine Definitions ---
AppState currentAppState = APP_STATE_IDLE;
extern IRState currentIRState;

// --- Global Variable Definitions ---
extern void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
extern void prepareTemperaturePlot();
extern void plotVoltageData();
extern void plotTemperatureData();
extern void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
extern void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
CurrentModel currentModel;
SHT4xSensor sht4Sensor;

extern void displayInternalResistanceGraph();
extern void drawChargePlot(bool autoscaleX, bool autoscaleY);
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile float mAh_charged = 0.0f;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;
uint32_t dutyCycle = 0;

unsigned long lastPlotUpdateTime = 0;
unsigned long lastDataGatherTime = 0;
unsigned long lastChargingHouseTime = 0;
unsigned long lastIRHandleTime = 0;
unsigned long displayStateChangeTime = 0;

const int pwmPin = PWM_PIN;
const int pwmResolutionBits = 8;
const int pwmMaxDutyCycle = (1 << pwmResolutionBits) - 1;
double THERMISTOR_1_OFFSET = 0.0;
volatile uint32_t voltage_last_time;
volatile uint32_t voltage_update_interval = 250;

// --- Function Implementations ---

void setupPWM() {
    analogWriteResolution(pwmPin,pwmResolutionBits);
    analogWriteFrequency(pwmPin,PWM_FREQUENCY);
    pinMode(pwmPin, OUTPUT);
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
}

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
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

// --- Non-blocking buildCurrentModel ---
int buildModelStep = 0;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
std::vector<float> dutyCycles;
std::vector<float> currents;

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

void buildCurrentModelStep() {
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
            dutyCycle = buildModelDutyCycle;
            analogWrite(pwmPin, dutyCycle);
            buildModelLastStepTime = now;
            buildModelStep = 2;
        } else {
            buildModelStep = 3;
        }
    }

    if (buildModelStep == 2) {
        if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
            MeasurementData data;
            getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
            data.dutyCycle = buildModelDutyCycle;
            data.timestamp = millis();

            processThermistorData(data, "Estimating Min Current");
            if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
                dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                currents.push_back(data.current);
            } else {
                Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", data.current, buildModelDutyCycle);
            }
            buildModelDutyCycle += 5;
            buildModelStep = 1;
        }
    }

    if (buildModelStep == 3) {
        if (dutyCycles.size() < 2) {
            Serial.println("Not enough data points to build a reliable model.");
            currentModel.isModelBuilt = false;
            dutyCycle = 0;
            analogWrite(pwmPin, dutyCycle);
            currentAppState = APP_STATE_IDLE;
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
        dutyCycle = 0;
        analogWrite(pwmPin, dutyCycle);
        currentAppState = APP_STATE_IDLE;
        startCharging();
        buildModelStep = 0;
    }
}

float estimateCurrent(int dutyCycle) {
    if (!currentModel.isModelBuilt) {
        Serial.println("Warning: Current model has not been built yet. Returning 0.");
        return 0.0f;
    }

    float estimatedCurrent = 0.0f;
    float dutyCycleFloat = static_cast<float>(dutyCycle);
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        estimatedCurrent += currentModel.coefficients(i) * std::pow(dutyCycleFloat, i);
    }

    if (estimatedCurrent < MEASURABLE_CURRENT_THRESHOLD && dutyCycle > 0) {
        Serial.printf("Estimated current (%.3f A) below threshold at duty cycle %d. Inferring.\n", estimatedCurrent, dutyCycle);
    }

    return std::max(0.0f, estimatedCurrent);
}

void task_readSHT4x(void* parameter) {
    while (true) {
        sht4Sensor.read();
        vTaskDelay(100);
    }
}

void task_readThermistor(void* parameter) {
    mAh_last_time = millis();

    while (true) {
        uint32_t current_time = millis();

        while (sht4Sensor.isLocked()) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        };

        thermistorSensor.read(sht4Sensor.getTemperature());

        int task_current_numSamples = 256;
        double sumAnalogValuesCurrent = 0;
        for (int i = 0; i < task_current_numSamples; ++i) {
            uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, CURRENT_SHUNT_OVERSAMPLING);
            sumAnalogValuesCurrent += analogValue;
        }
        double voltageAcrossShunt = (sumAnalogValuesCurrent / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE);

        if ((current_time - voltage_last_time) > voltage_update_interval) {
            int task_voltage_numSamples = 256;
            double sumAnalogValuesVoltage = 0;
            for (int i = 0; i < task_voltage_numSamples; ++i) {
                uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN, VOLTAGE_ATTENUATION, VOLTAGE_OVERSAMPLING);
                sumAnalogValuesVoltage += analogValue;
            }
            voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - (sumAnalogValuesVoltage / task_voltage_numSamples);
            voltage_last_time = current_time;
        }

        uint32_t time_elapsed = current_time - mAh_last_time;
        double time_elapsed_hours = (double)time_elapsed / (1000.0 * 3600.0);
        double current_for_mah_calculation_ma;

        if (currentModel.isModelBuilt && (current_ma / 1000.0) < MEASURABLE_CURRENT_THRESHOLD) {
            current_for_mah_calculation_ma = static_cast<double>(estimateCurrent(dutyCycle) * 1000.0);
        } else {
            current_for_mah_calculation_ma = current_ma;
        }

        mAh_charged += (current_for_mah_calculation_ma) * time_elapsed_hours;
        mAh_last_time = current_time;

        if (resetAh) {
            mAh_charged = 0.0f;
            resetAh = false;
            Serial.println("mAh counter reset.");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

#ifdef DEBUG_LABELS
#include <iostream>
int testGraph() {
    // Create some dummy data for testing
    for (int i = 0; i < 100; ++i) {
        chargeLog.push_back({(unsigned long)i * 1000, 0.2f + 0.1f * sin(i * 0.1f), 1.5f + 0.2f * cos(i * 0.05f), i % 256, 25.0f + 0.5f * i, 20.0f + 0.3f * i, 0.1f + 0.01f * i, 0.2f + 0.02f * i});
    }
    drawChargePlot(true, true);
    return 0;
}
#endif // #ifdef DEBUG_LABELS

void handleIRCommand() {
    if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x7) {
        Serial.print(F("Command 0x"));
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        switch(IrReceiver.decodedIRData.command) {
            case RemoteKeys::KEY_PLAY:
                currentAppState = APP_STATE_MEASURING_IR;
                currentIRState = IR_STATE_START;
                break;
            case RemoteKeys::KEY_INFO:
                if (currentAppState == APP_STATE_IDLE) {
                    currentDisplayState = DISPLAY_STATE_IR_GRAPH;
                    displayInternalResistanceGraph();
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_SOURCE:
                if (currentDisplayState != DISPLAY_STATE_CHARGE_GRAPH) {
                    currentDisplayState = DISPLAY_STATE_CHARGE_GRAPH;
                    drawChargePlot(true, true);
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_POWER:
                resetAh = true;
                currentAppState = APP_STATE_BUILDING_MODEL;
                break;

#ifdef DEBUG_LABELS

                case RemoteKeys::KEY_0:{
                testGraph();
                delay(20000); // wait 20 seconds
                tft.fillScreen(TFT_BLACK); // clear junk afterwards
                break;
                }
#endif // #ifdef DEBUG_LABELS                               
        }
    }
}

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    IrReceiver.begin(IR_RECEIVE_PIN);

    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f;
        current_values[i] = 0.0f;
    }

    sht4Sensor.begin();
    thermistorSensor.begin();

    xTaskCreate(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL);

    setupPWM();

    Serial.println("Ni-Cd/Ni-MH battery charger. use samsung DVD remote to control");
    Serial.println("PLAY to measure internal resistance of battery");
    Serial.println("INFO to see internal resistance graph");
    Serial.println("POWER to start charging");
    Serial.println("SOURCE to see charge graph");
}

void gatherData() {
    double temp1, temp2, tempDiff;
    float t1_millivolts, voltage, current;
    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
}

void updateDisplay() {
    if (currentDisplayState == DISPLAY_STATE_MAIN) {
        prepareTemperaturePlot();
        plotVoltageData();
        plotTemperatureData();

        displayTemperatureLabels(temp1_values[PLOT_WIDTH - 1], temp2_values[PLOT_WIDTH - 1], diff_values[PLOT_WIDTH - 1], 0, voltage_values[PLOT_WIDTH - 1], current_values[PLOT_WIDTH - 1]);

        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(19 * 10, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.print(mAh_charged, 3);
        tft.print(" mAh");
    }
}

void loop() {
    unsigned long now = millis();

    // --- State Machine ---
    switch (currentAppState) {
        case APP_STATE_IDLE:
            // Nothing to do
            break;
        case APP_STATE_BUILDING_MODEL:
            buildCurrentModelStep();
            break;
        case APP_STATE_MEASURING_IR:
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) {
                currentAppState = APP_STATE_IDLE;
            }
            break;
        case APP_STATE_CHARGING:
            if (!chargeBattery()) {
                currentAppState = APP_STATE_IDLE;
            }
            break;
    }

    // --- Timed Events ---
    if (now - lastDataGatherTime >= PLOT_DATA_UPDATE_INTERVAL) {
        lastDataGatherTime = now;
        gatherData();
    }

    if (now - lastPlotUpdateTime >= PLOT_UPDATE_INTERVAL_MS) {
        lastPlotUpdateTime = now;
        updateDisplay();
    }

    if (currentDisplayState != DISPLAY_STATE_MAIN && now - displayStateChangeTime > 20000) {
        currentDisplayState = DISPLAY_STATE_MAIN;
        tft.fillScreen(TFT_BLACK);
    }

    // --- IR Remote Handling ---
    if (now - lastIRHandleTime >= IR_HANDLE_INTERVAL_MS) {
        lastIRHandleTime = now;
        if (IrReceiver.decode()) {
            handleIRCommand();
            IrReceiver.resume();
        }
    }
}

//inline unsigned long unmanagedCastUL(unsigned long v){ return v; }
