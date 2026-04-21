#include "Arduino.h"
#include <iostream>
#include <cassert>

#include "../definitions.h"
#include "../home_screen.h"
#include "../internal_resistance.h"

unsigned long mock_millis = 0;
SerialMock Serial;
WiFiMock WiFi;

// 1. Definition of Globals (Matching .cpp files)
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

IRState nextIRState = IR_STATE_IDLE;
unsigned long irStateChangeTime = 0;
MeasurementData currentMeasurement;
int minimalDutyCycle = 0;
int findMinDcLow = 0;
int findMinDcHigh = 0;
int findMinDcMid = 0;
std::vector<std::pair<int, int>> dutyCyclePairs;
int pairIndex = 0;
int pairGenerationStep = 0;
int pairGenerationSubStep = 0;
int lowDc = 0;
int previousHighDc = 0;
int lowBound = 0;
int highBound = 0;
int bestHighDc = 0;
float minCurrent = 0.0f;
float maxCurrent = 0.0f;
float minCurrentDifference = 0.0f;
int measureStep = 0;
std::vector<float> voltagesLoaded;
std::vector<float> currentsLoaded;
std::vector<float> ir_dutyCycles;
std::vector<float> consecutiveInternalResistances;
bool isMeasuringResistance = false;

AsyncMeasure meas;
FindOptManager findOpt;
RemeasureManager remeasure;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.150;
float currentRampTarget = 0.0f;

float eval_mAh_snapshot = 0.0f;
unsigned long eval_time_snapshot = 0;
bool reeval_active = false;
float reeval_start_mAh = 0.0f;
unsigned long reeval_start_ms = 0;
float lastReeval_delta_mAh = 0.0f;
unsigned long lastReeval_duration_ms = 0;
float lastReeval_avgCurrent_A = 0.0f;

// Logic functions
void applyDuty(uint32_t duty) {
    dutyCycle = duty;
    current_ma = (float)(duty * 2.0f);
}
void getThermistorReadings(double& t1, double& t2, double& td, float& t1mv, float& v, float& i) {
    t1 = 24.5; t2 = 25.0; td = 0.5;
    v = 1.2; i = current_ma / 1000.0f;
}
float estimateCurrent(int duty) { return duty * 0.002f; }
int estimateDutyCycleForCurrent(float target) { return (int)(target / 0.002f); }

// Hardware stubs
SystemDataManager::SystemDataManager(SHT4xSensor& sht, int p1, int vcc, double off) : _sht4(sht) {}
void SystemDataManager::begin() {}
void SystemDataManager::update() {}
void SystemDataManager::resetMah() {}
SystemData SystemDataManager::getData() {
    SystemData d;
    d.battery_voltage_v = 1.2f; d.charge_current_a = current_ma / 1000.0f;
    d.ambient_temp_c = 24.5f; d.battery_temp_c = 25.0f; d.temp_diff_c = 0.5f;
    d.mah_charged = (float)mAh_charged; d.vcc_mv = 3300.0f;
    return d;
}
SystemDataManager systemData(sht4Sensor, 0, 0, 0);

SHT4xSensor::SHT4xSensor() {}
HomeScreen::HomeScreen() {}
void HomeScreen::begin() {}
void HomeScreen::gatherData() {}

// Logic Inclusion (Unity build)
#include "../web_handlers.cpp"
#include "../logging.cpp"
#include "../internal_resistance.cpp"
#include "../charging.cpp"

void buildCurrentModelStep() {
    currentModel.isModelBuilt = true;
    currentAppState = APP_STATE_IDLE;
    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_IDLE;
    applyDuty(100);
}

void advanceSimulation(int steps, unsigned long ms_per_step) {
    for (int i = 0; i < steps; i++) {
        mock_millis += ms_per_step;
        if (current_ma > 0) mAh_charged += (current_ma * ms_per_step) / 3600000.0;
        if (currentAppState == APP_STATE_BUILDING_MODEL) buildCurrentModelStep();
        if (currentAppState == APP_STATE_CHARGING) chargeBattery();
    }
}

void testThermalModel() {
    std::cout << "Testing Thermal Model (estimateTempDiff)..." << std::endl;
    // float estimateTempDiff(float vL, float vNL, float cur, float R, float amb, uint32_t now, uint32_t last, float bat, ...)
    float vL = 1.2f;
    float vNL = 1.3f;
    float cur = 0.1f;
    float R = 0.1f;
    float amb = 25.0f;
    float bat = 25.0f;
    uint32_t now = 1000;
    uint32_t last = 0;

    float theta = estimateTempDiff(vL, vNL, cur, R, amb, now, last, bat);
    std::cout << "Theta rise after 1s: " << theta << std::endl;
    assert(theta >= 0); // Should have some rise or stay same
    std::cout << "Thermal model test passed!" << std::endl;
}

int main() {
    std::cout << "Starting Advanced Simulator Tests..." << std::endl;

    testThermalModel();

    server.args["cmd"] = "charge";
    handleCommand();
    advanceSimulation(100, 1000);
    std::cout << "State: " << (int)currentAppState << ", mAh: " << mAh_charged << std::endl;
    assert(currentAppState == APP_STATE_CHARGING);
    assert(mAh_charged > 0);

    std::cout << "All simulation tests passed!" << std::endl;
    return 0;
}
