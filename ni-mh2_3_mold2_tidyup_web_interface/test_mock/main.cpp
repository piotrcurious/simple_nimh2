#include "Arduino.h"
#include <iostream>
#include <cassert>

// Define missing types/enums for web_handlers.cpp
enum IRState {
    IR_STATE_IDLE,
    IR_STATE_START,
    IR_STATE_STOP_LOAD_WAIT,
    IR_STATE_GET_UNLOADED_VOLTAGE,
    IR_STATE_FIND_MIN_DC,
    IR_STATE_GENERATE_PAIRS,
    IR_STATE_MEASURE_L_UL,
    IR_STATE_MEASURE_PAIRS,
    IR_STATE_GET_MEASUREMENT,
    IR_STATE_COMPLETE
};
IRState currentIRState = IR_STATE_IDLE;

// Include core logic
#include "../definitions.h"
#include "../home_screen.h"

SerialMock Serial;
WiFiMock WiFi;

// Stubs and Globals
float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;
std::vector<ChargeLogData> chargeLog;

// Global symbols from main .ino
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
uint32_t dutyCycle = 0;
CurrentModel currentModel;
SHT4xSensor sht4Sensor;
HomeScreen homeScreen;
WebServer server(80);

float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];
float voltage_values[PLOT_WIDTH];
float current_values[PLOT_WIDTH];

void applyDuty(uint32_t duty) { dutyCycle = duty; }
void logChargeData(const ChargeLogData& data) { chargeLog.push_back(data); }

// Stub implementations for classes
SHT4xSensor::SHT4xSensor() {}
double SHT4xSensor::getTemperature() { return 25.0; }
double SHT4xSensor::getHumidity() { return 50.0; }
HomeScreen::HomeScreen() {}

// Link in external logic
#include "../web_handlers.cpp"

void testJsonGeneration() {
    std::cout << "Testing JSON generation..." << std::endl;
    voltage_mv = 1234.0f;
    current_ma = 150.0f;
    mAh_charged = 10.5f;
    dutyCycle = 128;
    currentAppState = APP_STATE_CHARGING;

    String stateJson = getJsonState();
    std::cout << "State JSON: " << stateJson << std::endl;
    assert(stateJson.find("\"app\":3") != std::string::npos);
    assert(stateJson.find("\"v\":1.234") != std::string::npos);
    assert(stateJson.find("\"mah\":10.500") != std::string::npos);

    chargeLog.clear();
    chargeLog.push_back({1000, 0.15f, 1.25f, 25.0f, 30.0f, 100, 0.1f, 0.12f});
    String logJson = getJsonChargeLog();
    std::cout << "Log JSON: " << logJson << std::endl;
    assert(logJson.find("\"i\":0.150") != std::string::npos);

    std::cout << "JSON tests passed!" << std::endl;
}

void testCommandHandling() {
    std::cout << "Testing command handling..." << std::endl;
    server.args["cmd"] = "stop";
    handleCommand();
    assert(currentAppState == APP_STATE_IDLE);
    assert(dutyCycle == 0);

    server.args["cmd"] = "charge";
    handleCommand();
    assert(currentAppState == APP_STATE_BUILDING_MODEL);
    assert(resetAh == true);

    std::cout << "Command tests passed!" << std::endl;
}

int main() {
    std::cout << "Starting Mock Arduino Test Runner" << std::endl;
    testJsonGeneration();
    testCommandHandling();
    std::cout << "All tests passed successfully!" << std::endl;
    return 0;
}
