#include "Arduino.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <iomanip>

#ifndef MOCK_TEST
#define MOCK_TEST
#endif

#include "../definitions.h"
#include "../home_screen.h"
#include "../internal_resistance.h"

unsigned long mock_millis = 0;
SerialMock Serial;
WiFiMock WiFi;

// --- Global Variables (Matching the project) ---
float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.2f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.2f;
std::vector<ChargeLogData> chargeLog;
unsigned long chargingStartTime = 0;

IRState currentIRState = IR_STATE_IDLE;
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;
volatile float voltage_mv = 1200.0f;
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

// Internal states for IR and Charging
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
int cachedOptimalDuty = 100;
unsigned long chargePhaseStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
float maximumCurrent = 0.500;
float currentRampTarget = 0.0f;

float eval_mAh_snapshot = 0.0f;
unsigned long eval_time_snapshot = 0;
bool reeval_active = false;
float reeval_start_mAh = 0.0f;
unsigned long reeval_start_ms = 0;
float lastReeval_delta_mAh = 0.0f;
unsigned long lastReeval_duration_ms = 0;
float lastReeval_avgCurrent_A = 0.0f;

// --- Mock Battery Physics ---
float battery_ocv = 1.25f;
float battery_ir = 0.15f;
float current_sense_factor = 0.003f;
float battery_temp = 24.0f;
float ambient_temp = 24.0f;
float thermal_mass = 50.0f; // Simplified thermal mass
float convection_coeff = 0.01f;

void applyDuty(uint32_t duty) {
    dutyCycle = duty;
    current_ma = (float)(duty * current_sense_factor * 1000.0f);
    // Simple V = OCV + I*R
    voltage_mv = (battery_ocv + (current_ma / 1000.0f) * battery_ir) * 1000.0f;
}

void updatePhysics(unsigned long step_ms) {
    float current_a = current_ma / 1000.0f;
    float power_w = current_a * current_a * battery_ir;
    float heat_loss = convection_coeff * (battery_temp - ambient_temp);
    float dt = (power_w - heat_loss) / thermal_mass * (step_ms / 1000.0f);
    battery_temp += dt;
}

void getThermistorReadings(double& t1, double& t2, double& td, float& t1mv, float& v, float& i) {
    t1 = ambient_temp;
    t2 = battery_temp;
    td = t2 - t1;
    t1mv = 0;
    v = voltage_mv / 1000.0f;
    i = current_ma / 1000.0f;
}

// --- Mock Hardware Stubs ---
SystemDataManager::SystemDataManager(SHT4xSensor& sht, int p1, int vcc, double off) : _sht4(sht) {}
void SystemDataManager::begin() {}
void SystemDataManager::update() {
    if (resetAh) { mAh_charged = 0; resetAh = false; }
}
void SystemDataManager::resetMah() { mAh_charged = 0; }
SystemData SystemDataManager::getData() {
    SystemData d;
    d.battery_voltage_v = voltage_mv / 1000.0f;
    d.charge_current_a = current_ma / 1000.0f;
    d.ambient_temp_c = 24.0f;
    d.battery_temp_c = 24.0f + (current_ma / 500.0f);
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

// --- Simple Linear Regression Implementation for Mock ---
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) return false;
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < count; i++) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += data[i][0] * data[i][1];
        sumX2 += data[i][0] * data[i][0];
    }
    double denominator = (count * sumX2 - sumX * sumX);
    if (std::abs(denominator) < 1e-9) return false;
    slope = (float)((count * sumXY - sumX * sumY) / denominator);
    intercept = (float)((sumY - slope * sumX) / count);
    return true;
}

// --- Logic Inclusion ---
#include "../web_handlers.cpp"
#include "../logging.cpp"
#include "../internal_resistance.cpp"
#include "../charging.cpp"
#include "../graphing.cpp"

float estimateCurrent(int duty) { return duty * current_sense_factor; }
int estimateDutyCycleForCurrent(float target) { return (int)(target / current_sense_factor); }

// --- Simulation Helpers ---
void advanceSimulation(unsigned long ms) {
    unsigned long target = mock_millis + ms;
    const unsigned long step = 50;
    while (mock_millis < target) {
        mock_millis += step;
        if (current_ma > 0) {
            mAh_charged += (current_ma * step) / 3600000.0;
        }

        updatePhysics(step);

        if (currentAppState == APP_STATE_MEASURING_IR) {
            measureInternalResistanceStep();
        } else if (currentAppState == APP_STATE_CHARGING) {
            chargeBattery();
        }
    }
}

// --- Test Cases ---

void testInternalResistanceGranular() {
    std::cout << "--- Testing IR State Transitions ---" << std::endl;
    resistanceDataCount = 0;
    currentAppState = APP_STATE_MEASURING_IR;
    currentIRState = IR_STATE_START;

    measureInternalResistanceStep();
    assert(currentIRState == IR_STATE_STOP_LOAD_WAIT);
    std::cout << "Transition to STOP_LOAD_WAIT OK" << std::endl;

    mock_millis += UNLOADED_VOLTAGE_DELAY_MS;
    measureInternalResistanceStep();
    assert(currentIRState == IR_STATE_GET_UNLOADED_VOLTAGE);
    std::cout << "Transition to GET_UNLOADED_VOLTAGE OK" << std::endl;

    measureInternalResistanceStep();
    assert(currentIRState == IR_STATE_GET_MEASUREMENT);
    assert(nextIRState == IR_STATE_FIND_MIN_DC);
    std::cout << "Transition to GET_MEASUREMENT (FIND_MIN_DC) OK" << std::endl;

    mock_millis += STABILIZATION_DELAY_MS;
    measureInternalResistanceStep();
    assert(currentIRState == IR_STATE_FIND_MIN_DC);
    std::cout << "Back to FIND_MIN_DC OK" << std::endl;

    // Fast forward to complete
    int safety = 0;
    while(currentAppState == APP_STATE_MEASURING_IR && safety < 10000) {
        measureInternalResistanceStep();
        if (currentIRState == IR_STATE_GET_MEASUREMENT) mock_millis += STABILIZATION_DELAY_MS;
        else if (currentIRState == IR_STATE_STOP_LOAD_WAIT) mock_millis += UNLOADED_VOLTAGE_DELAY_MS;
        else mock_millis += 1;
        safety++;
    }
    assert(currentIRState == IR_STATE_IDLE);
    assert(resistanceDataCount > 0);
    std::cout << "IR Measurement Full Cycle OK" << std::endl;
}

void testChargingGranular() {
    std::cout << "--- Testing Charging Logic Granular ---" << std::endl;
    chargingState = CHARGE_IDLE;
    currentAppState = APP_STATE_CHARGING;
    currentModel.isModelBuilt = true;

    chargeBattery();
    assert(chargingState == CHARGE_FIND_OPT);
    assert(findOpt.active == true);
    assert(findOpt.phase == FIND_INIT_HIGHDC);
    std::cout << "Charging Init -> FIND_INIT_HIGHDC OK" << std::endl;

    // Simulate one measurement result for findOpt
    int safety = 0;
    while(findOpt.phase == FIND_INIT_HIGHDC && safety < 100) {
        chargeBattery();
        if (meas.active()) mock_millis += 4000;
        else mock_millis += 100;
        safety++;
    }
    assert(findOpt.phase == FIND_BINARY_PREPARE);
    std::cout << "Transition to FIND_BINARY_PREPARE OK" << std::endl;

    // Fast forward to MONITOR
    safety = 0;
    while(chargingState == CHARGE_FIND_OPT && safety < 5000) {
        chargeBattery();
        if (meas.active()) {
            if (meas.state == MEAS_STOPLOAD_WAIT) mock_millis += UNLOADED_VOLTAGE_DELAY_MS;
            else if (meas.state == MEAS_APPLY_LOAD) mock_millis += STABILIZATION_DELAY_MS;
            // No need to manually set resultReady/COMPLETE as measurementStep() does it when time passes
        } else {
            mock_millis += 10;
        }
        safety++;
    }
    std::cout << "Final chargingState: " << (int)chargingState << " safety: " << safety << std::endl;
    assert(chargingState == CHARGE_MONITOR);
    std::cout << "Transition to CHARGE_MONITOR OK" << std::endl;

    // Trigger Re-evaluation
    eval_time_snapshot = mock_millis - CHARGE_EVALUATION_INTERVAL_MS - 100;
    chargeBattery();
    assert(chargingState == CHARGE_FIND_OPT);
    assert(findOpt.isReevaluation == true);
    std::cout << "Re-evaluation triggered OK" << std::endl;
}

void testSafetyOverTemperature() {
    std::cout << "--- Testing Safety: Over-Temperature Shutdown ---" << std::endl;
    battery_temp = 24.0f;
    ambient_temp = 24.0f;
    battery_ir = 0.15f;
    currentAppState = APP_STATE_CHARGING;
    currentModel.isModelBuilt = true;
    chargingState = CHARGE_MONITOR;
    dutyCycle = 100;
    applyDuty(dutyCycle);

    // Ensure currentRampTarget is high enough
    currentRampTarget = maximumCurrent;
    // Set last evaluation time to long ago to trigger check immediately
    eval_time_snapshot = mock_millis - CHARGE_EVALUATION_INTERVAL_MS - 1;

    // Artificially heat the battery
    battery_temp = ambient_temp + 20.0f;

    // We need 3 trips (OVERTEMP_TRIP_TRESHOLD)
    for(int i=0; i<4; i++) {
        eval_time_snapshot = mock_millis - CHARGE_EVALUATION_INTERVAL_MS - 1;
        chargeBattery();
        // After each evaluation, it might go back to FIND_OPT if re-evaluation is triggered
        if (chargingState == CHARGE_FIND_OPT) {
            // Fast forward FIND_OPT
            while(chargingState == CHARGE_FIND_OPT) {
                chargeBattery();
                mock_millis += 1000;
            }
        }
    }

    assert(chargingState == CHARGE_STOPPED || dutyCycle == 0);
    std::cout << "Safety Shutdown OK" << std::endl;
}

void testVoltageDipBehavior() {
    std::cout << "--- Testing Behavior: Voltage Dip ---" << std::endl;
    battery_ocv = 1.4f;
    battery_temp = 24.0f;
    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_MONITOR;
    dutyCycle = 100;
    applyDuty(dutyCycle);

    float initial_v = voltage_mv;
    battery_ocv = 1.1f; // Simulated dip (e.g. cell internal short or contact issue)
    applyDuty(dutyCycle);

    assert(voltage_mv < initial_v);
    std::cout << "Voltage Dip Simulated: " << initial_v << " -> " << voltage_mv << " mV" << std::endl;

    // Check if system continues or handles it
    chargeBattery();
    assert(chargingState == CHARGE_MONITOR || chargingState == CHARGE_FIND_OPT);
    std::cout << "System survived dip (transitions as expected) OK" << std::endl;
}

void testAPIJSONRobustness() {
    std::cout << "--- Testing API JSON Robustness ---" << std::endl;

    // Test with Inf
    for(int i=0; i<PLOT_WIDTH; i++) voltage_values[i] = std::numeric_limits<float>::infinity();
    String hist = getJsonHistory();
    assert(hist.indexOf("null") == -1); // float(inf) usually becomes "inf" in Arduino String, but we want to see if it crashes
    // Actually std::stringstream in our mock might output "inf"

    // Test Charge Log with many entries
    chargeLog.clear();
    for(int i=0; i<500; i++) {
        ChargeLogData e = { (unsigned long)i*1000, 0.2f, 1.3f, 25.0f, 26.0f, 100, 0.2f, 0.18f };
        chargeLog.push_back(e);
    }
    String clog = getJsonChargeLog();
    assert(clog.length() > 1000);
    std::cout << "JSON Robustness OK" << std::endl;
}

int main() {
    std::cout << "Starting Granular Mock Tests..." << std::endl;

    testAPIJSONRobustness();
    testInternalResistanceGranular();
    testChargingGranular();
    testSafetyOverTemperature();
    testVoltageDipBehavior();

    std::cout << "All stress and granular tests passed successfully!" << std::endl;
    return 0;
}
