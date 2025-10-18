#include "definitions.h"
#include "logging.h"
#include "DataPlotter.h"
#include "Charger.h"
#include "internal_resistance.h"
#include <IRremote.h>
#include <TFT_eSPI.h>

// --- Global Objects ---
TFT_eSPI tft = TFT_eSPI();
DataPlotter dataPlotter(tft);
SHT4xSensor sht4Sensor;
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);
Charger charger(PWM_PIN, dataPlotter);

// --- State Machine Definitions ---
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_MAIN;
extern IRState currentIRState;

// --- Global Variable Definitions ---
// For thermistor readings shared between data gathering and display
double latest_temp1, latest_temp2, latest_tempDiff;
float latest_t1_millivolts, latest_voltage, latest_current;

volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile float mAh_charged = 0.0f;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;

unsigned long lastPlotUpdateTime = 0;
unsigned long lastDataGatherTime = 0;
unsigned long lastIRHandleTime = 0;
unsigned long displayStateChangeTime = 0;

double THERMISTOR_1_OFFSET = 0.0;
volatile uint32_t voltage_last_time;
volatile uint32_t voltage_update_interval = 250;

// --- Function Implementations ---

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

        if (charger.isModelBuilt() && (current_ma / 1000.0) < MEASURABLE_CURRENT_THRESHOLD) {
            current_for_mah_calculation_ma = static_cast<double>(charger.estimateCurrent(charger.getDutyCycle()) * 1000.0);
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
    extern std::vector<ChargeLogData> chargeLog;
    extern float regressedInternalResistancePairsIntercept;
    for (int i = 0; i < 100; ++i) {
        chargeLog.push_back({(unsigned long)i * 1000, 0.2f + 0.1f * sin(i * 0.1f), 1.5f + 0.2f * cos(i * 0.05f), (uint8_t)(i % 256), 25.0f + 0.5f * i, 20.0f + 0.3f * i, 0.1f + 0.01f * i, 0.2f + 0.02f * i});
    }
    dataPlotter.drawChargePlot(true, true, chargeLog, regressedInternalResistancePairsIntercept);
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
                 if (currentAppState == APP_STATE_IDLE || currentAppState == APP_STATE_CHARGING ) {
                    currentDisplayState = DISPLAY_STATE_IR_GRAPH;
                    extern float internalResistanceData[MAX_RESISTANCE_SAMPLES][2];
                    extern int resistanceDataCount;
                    extern float internalResistanceDataPairs[MAX_RESISTANCE_SAMPLES][2];
                    extern int resistanceDataCountPairs;
                    extern float regressedInternalResistanceSlope;
                    extern float regressedInternalResistanceIntercept;
                    extern float regressedInternalResistancePairsSlope;
                    extern float regressedInternalResistancePairsIntercept;
                    dataPlotter.displayInternalResistanceGraph(internalResistanceData, resistanceDataCount, internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistanceSlope, regressedInternalResistanceIntercept, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_SOURCE:
                if (currentDisplayState != DISPLAY_STATE_CHARGE_GRAPH) {
                    currentDisplayState = DISPLAY_STATE_CHARGE_GRAPH;
                    extern std::vector<ChargeLogData> chargeLog;
                    extern float regressedInternalResistancePairsIntercept;
                    dataPlotter.drawChargePlot(true, true, chargeLog, regressedInternalResistancePairsIntercept);
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_POWER:
                resetAh = true;
                charger.startModelBuilding();
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

    sht4Sensor.begin();
    thermistorSensor.begin();

    xTaskCreate(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL);

    charger.begin();

    Serial.println("Ni-Cd/Ni-MH battery charger. use samsung DVD remote to control");
    Serial.println("PLAY to measure internal resistance of battery");
    Serial.println("INFO to see internal resistance graph");
    Serial.println("POWER to start charging");
    Serial.println("SOURCE to see charge graph");
}

void gatherData() {
    getThermistorReadings(latest_temp1, latest_temp2, latest_tempDiff, latest_t1_millivolts, latest_voltage, latest_current);
    dataPlotter.printThermistorSerial(latest_temp1, latest_temp2, latest_tempDiff, latest_t1_millivolts, latest_voltage, latest_current);
    dataPlotter.updateTemperatureHistory(latest_temp1, latest_temp2, latest_tempDiff, latest_voltage, latest_current);
}

void updateDisplay() {
    if (currentDisplayState == DISPLAY_STATE_MAIN) {
        dataPlotter.prepareTemperaturePlot();
        dataPlotter.plotVoltageData();
        dataPlotter.plotTemperatureData();
        dataPlotter.displayTemperatureLabels(latest_temp1, latest_temp2, latest_tempDiff, latest_t1_millivolts, latest_voltage, latest_current, thermistorSensor, charger.getDutyCycle());

        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(19 * 10, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.print(mAh_charged, 3);
        tft.print(" mAh");
    }
}

void loop() {
    unsigned long now = millis();

    charger.update();

    // --- State Machine for non-charging states ---
    switch (currentAppState) {
        case APP_STATE_IDLE:
            // Nothing to do
            break;
        case APP_STATE_MEASURING_IR:
            extern void measureInternalResistanceStep();
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) {
                currentAppState = APP_STATE_IDLE;
            }
            break;
        default:
            // The charger handles BUILDING_MODEL and CHARGING states.
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
        portMUX_TYPE ir_mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&ir_mux);
        bool is_ir_data = IrReceiver.decode();
        portEXIT_CRITICAL(&ir_mux);
        if (is_ir_data) {
            handleIRCommand();
            IrReceiver.resume();
        }
    }
}