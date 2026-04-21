#include "Arduino.h"
#include <iostream>
#include <cassert>

// Define missing types/enums
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

#include "../definitions.h"
#include "../home_screen.h"

unsigned long mock_millis = 0;
SerialMock Serial;
WiFiMock WiFi;

// Physics / Simulation State
float simulated_battery_voltage = 1.2f;
float simulated_battery_temp = 25.0f;
float simulated_ambient_temp = 24.5f;

// Persistence storage
float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.1f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.1f;
std::vector<ChargeLogData> chargeLog;
unsigned long chargingStartTime = 0;

IRState currentIRState = IR_STATE_IDLE;
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

// Logic functions
void applyDuty(uint32_t duty) {
    dutyCycle = duty;
    current_ma = (float)(duty * 2.0f);
}
void getThermistorReadings(double& t1, double& t2, double& td, float& t1mv, float& v, float& i) {
    t1 = simulated_ambient_temp; t2 = simulated_battery_temp; td = t2 - t1;
    v = simulated_battery_voltage; i = current_ma / 1000.0f;
}
float estimateCurrent(int duty) { return duty * 0.002f; }
int estimateDutyCycleForCurrent(float target) { return (int)(target / 0.002f); }

// Logic inclusion
// For the mock test to work, we avoid including .cpp that define their own globals
// or we make sure they are compatible.
#include "../web_handlers.cpp"
#include "../logging.cpp"

// Hardware stubs
SystemDataManager::SystemDataManager(SHT4xSensor& sht, int p1, int vcc, double off) : _sht4(sht) {}
void SystemDataManager::begin() {}
void SystemDataManager::update() {}
void SystemDataManager::resetMah() {}
SystemData SystemDataManager::getData() {
    SystemData d;
    d.battery_voltage_v = simulated_battery_voltage;
    d.charge_current_a = current_ma / 1000.0f;
    d.ambient_temp_c = simulated_ambient_temp;
    d.battery_temp_c = simulated_battery_temp;
    d.temp_diff_c = d.battery_temp_c - d.ambient_temp_c;
    d.mah_charged = (float)mAh_charged;
    d.vcc_mv = 3300.0f;
    return d;
}
SystemDataManager systemData(sht4Sensor, 0, 0, 0);

SHT4xSensor::SHT4xSensor() {}
HomeScreen::HomeScreen() {}
void HomeScreen::begin() {}
void HomeScreen::gatherData() {}

void buildCurrentModelStep() {
    currentModel.isModelBuilt = true;
    currentAppState = APP_STATE_IDLE;
}

void startCharging() {
    currentAppState = APP_STATE_CHARGING;
}

// Tests
void testLifecycle() {
    std::cout << "Starting Simulation Lifecycle..." << std::endl;
    currentAppState = APP_STATE_IDLE;

    server.args["cmd"] = "charge";
    handleCommand();
    assert(currentAppState == APP_STATE_BUILDING_MODEL);

    buildCurrentModelStep();
    assert(currentAppState == APP_STATE_IDLE);

    server.args["cmd"] = "stop";
    handleCommand();
    assert(currentAppState == APP_STATE_IDLE);
    assert(dutyCycle == 0);

    std::cout << "Lifecycle test passed!" << std::endl;
}

int main() {
    std::cout << "=== Battery Charger Simulator v4 ===" << std::endl;
    testLifecycle();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
