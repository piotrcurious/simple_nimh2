#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>

#include "dummy_esp32.h"

// Math helpers
using std::isnan;
using std::isfinite;
using std::min;
using std::max;

MockSerial Serial;
unsigned long mock_millis = 0;

// Redirect definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define Adafruit_SHT4x_h
#define ADC_CONTINUOUS_H
#define ESP_ADC_CAL_H
#define ADC_H
#define WEBSERVER_H

#include "../ni-mh2_3_mold2_tidyup_web_interface/definitions.h"
#include "../ni-mh2_3_mold2_tidyup_web_interface/internal_resistance.h"

// Hardware stubs that would normally be in .ino
void applyDuty(uint32_t duty);
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
float estimateCurrent(int duty);
int estimateDutyCycleForCurrent(float target);

// Physics Simulation
struct BatterySim {
    float voltage = 1.15f;
    float temp = 22.0f;
    float ambient = 22.0f;
    float soc = 0.05f;
    float internal_resistance = 0.15f;
    float capacity_ah = 2.0f;

    // Non-linear duty-to-current mapping (simulates transistor behavior)
    float getCurrent(int duty) {
        if (duty < 20) return 0.0f;
        float normalized = (duty - 20) / 235.0f;
        return 2.5f * (normalized * normalized);
    }

    void update(float dt_s, int duty) {
        float current = getCurrent(duty);
        soc += (current * dt_s) / (capacity_ah * 3600.0f);

        float base_v = 1.2f + 0.15f * std::pow(soc, 0.5f);
        if (soc > 0.9f) base_v += (soc - 0.9f) * 4.0f;

        float current_ir = internal_resistance * (1.0f + std::max(0.0f, (float)std::abs(0.5f - soc) * 0.5f));
        voltage = base_v + current * current_ir;

        float efficiency = 1.0f;
        if (soc > 0.8f) efficiency = 1.0f - (soc - 0.8f) * 2.0f;
        if (efficiency < 0) efficiency = 0;
        float P_heat = current * current * current_ir + current * voltage * (1.0f - efficiency);

        float dT = (P_heat * dt_s) / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT);
        float cooling = (temp - ambient) * 0.05f * dt_s;
        temp += dT - cooling;
    }
};
BatterySim sim;

// Global variables from .ino
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;
uint32_t dutyCycle = 0;
bool isCharging = false;
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;
unsigned long lastPlotUpdateTime = 0;
unsigned long lastChargingHouseTime = 0;
const int pwmPin = 19;
double THERMISTOR_1_OFFSET = 0.0;

void applyDuty(uint32_t duty) { dutyCycle = duty; }

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    temp1 = sim.ambient;
    temp2 = sim.temp;
    tempDiff = temp2 - temp1;
    t1_millivolts = 0;
    voltage = sim.voltage;
    current = sim.getCurrent(dutyCycle);
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
}

#include "../ni-mh2_3_mold2_tidyup_web_interface/SHT4xSensor.h"
SHT4xSensor::SHT4xSensor() : _temperature(22.0f), _humidity(50.0f) {}
bool SHT4xSensor::begin() { return true; }
void SHT4xSensor::read() {}
void SHT4xSensor::setPrecision(sht4x_precision_t p) {}
void SHT4xSensor::setHeater(sht4x_heater_t h) {}

SystemDataManager::SystemDataManager(SHT4xSensor& s, int p1, int p2, double o) : _sht4(s) {
    _dataMutex = xSemaphoreCreateMutex();
}
void SystemDataManager::begin() {}
void SystemDataManager::update() {
    static uint32_t last_m = 0;
    uint32_t now = mock_millis;
    if (last_m == 0) last_m = now;
    float dt_h = (float)(now - last_m) / 3600000.0f;
    mAh_charged += (current_ma * dt_h);
    last_m = now;
}
void SystemDataManager::resetMah() { mAh_charged = 0; }
SystemData SystemDataManager::getData() {
    return { (float)voltage_mv/1000.0f, (float)current_ma/1000.0f, (double)sim.ambient, (double)sim.temp, (double)(sim.temp-sim.ambient), 5000.0f, (float)mAh_charged, (uint32_t)mock_millis };
}

SHT4xSensor sht4Sensor;
SystemDataManager systemData(sht4Sensor, 36, 35, 0.0);
CurrentModel currentModel;

#define private public
#include "../ni-mh2_3_mold2_tidyup_web_interface/home_screen.h"
#include "../ni-mh2_3_mold2_tidyup_web_interface/home_screen.cpp"
#undef private
HomeScreen homeScreen;

// Include logic
#include "../ni-mh2_3_mold2_tidyup_web_interface/charging.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/internal_resistance.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/graphing.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/logging.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/web_handlers.cpp"

// Manually bring in parts of .ino for testing model build
enum class BuildModelPhase { Idle = 0, Start, SetDuty, WaitMeasurement, Finish };
BuildModelPhase buildModelPhase = BuildModelPhase::Idle;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
std::vector<float> mock_dutyCycles;
std::vector<float> mock_currents;

void buildCurrentModelStep() {
    const unsigned long now = millis();
    switch (buildModelPhase) {
        case BuildModelPhase::Idle:
            mock_dutyCycles.clear(); mock_currents.clear();
            mock_dutyCycles.push_back(0.0f); mock_currents.push_back(0.0f);
            buildModelDutyCycle = 1; buildModelPhase = BuildModelPhase::SetDuty;
            break;
        case BuildModelPhase::SetDuty:
            if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
                applyDuty(buildModelDutyCycle);
                buildModelLastStepTime = now;
                buildModelPhase = BuildModelPhase::WaitMeasurement;
            } else buildModelPhase = BuildModelPhase::Finish;
            break;
        case BuildModelPhase::WaitMeasurement:
            if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
                double t1, t2, td; float tmv, v, c; getThermistorReadings(t1, t2, td, tmv, v, c);
                if (c >= MEASURABLE_CURRENT_THRESHOLD) {
                    mock_dutyCycles.push_back((float)buildModelDutyCycle);
                    mock_currents.push_back(c);
                }
                buildModelDutyCycle += 5; buildModelPhase = BuildModelPhase::SetDuty;
            }
            break;
        case BuildModelPhase::Finish:
            if (mock_dutyCycles.size() >= 2) {
                int n = (int)mock_dutyCycles.size();
                int degree = 3;
                Eigen::MatrixXd A(n, degree + 1);
                Eigen::VectorXd b(n);
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j <= degree; j++) A(i, j) = std::pow(mock_dutyCycles[i], j);
                    b(i) = mock_currents[i];
                }
                currentModel.coefficients = A.householderQr().solve(b);
                if (degree >= 0) currentModel.coefficients(0) = 0.0;
                currentModel.isModelBuilt = true;
                applyDuty(0);
                currentAppState = APP_STATE_IDLE; // Reset to idle
                startCharging(); // Auto start charging as in .ino
            } else {
                currentAppState = APP_STATE_IDLE;
            }
            buildModelPhase = BuildModelPhase::Idle;
            break;
    }
}

float estimateCurrent(int duty) {
    if (!currentModel.isModelBuilt) return sim.getCurrent(duty);
    double sum = 0.0;
    for (int i = 0; i < currentModel.coefficients.size(); ++i) sum += currentModel.coefficients(i) * std::pow((float)duty, i);
    return (float)std::max(0.0, sum);
}

int estimateDutyCycleForCurrent(float targetCurrent) {
    int bestDC = 0;
    float closestCurrentDiff = std::numeric_limits<float>::max();
    for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; ++dc) {
        float estimated = estimateCurrent(dc);
        float diff = std::abs(estimated - targetCurrent);
        if (diff < closestCurrentDiff) { closestCurrentDiff = diff; bestDC = dc; }
    }
    return bestDC;
}

WebServer server;

void reset_globals() {
    mock_millis = 0; voltage_mv = 1000.0f; current_ma = 0.0f; mAh_charged = 0.0;
    dutyCycle = 0; chargingState = CHARGE_IDLE; chargeLog.clear();
    sim = BatterySim(); overtemp_trip_counter = 0; currentAppState = APP_STATE_IDLE;
    currentIRState = IR_STATE_IDLE; isMeasuringResistance = false; isCharging = false;
    recentChargeLogsCount = 0; recentChargeLogsHead = 0;
    currentModel.isModelBuilt = false;
    server.args.clear();
    homeScreen.begin();
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = NAN;
        temp2_values[i] = NAN;
        diff_values[i] = NAN;
        voltage_values[i] = NAN;
        current_values[i] = NAN;
    }
}

void test_model_accuracy() {
    printf("Running test_model_accuracy...\n");
    reset_globals();
    currentAppState = APP_STATE_BUILDING_MODEL;
    buildModelPhase = BuildModelPhase::Idle;
    while (currentAppState == APP_STATE_BUILDING_MODEL) {
        mock_millis += 10;
        buildCurrentModelStep();
        if (buildModelPhase == BuildModelPhase::WaitMeasurement) mock_millis += BUILD_CURRENT_MODEL_DELAY;
    }
    assert(currentModel.isModelBuilt);
    float sum_sq_error = 0;
    int count = 0;
    for (int dc = 0; dc <= 255; dc += 10) {
        float actual = sim.getCurrent(dc);
        float estimated = estimateCurrent(dc);
        float error = actual - estimated;
        sum_sq_error += error * error;
        count++;
    }
    float rmse = std::sqrt(sum_sq_error / count);
    printf("  RMSE: %.4f\n", rmse);
    assert(rmse < 0.05);

    // Test inverse mapping
    float target = 1.0f;
    int dc = estimateDutyCycleForCurrent(target);
    float est = estimateCurrent(dc);
    printf("  Target 1.0A -> Duty %d -> Estimated %.3fA\n", dc, est);
    assert(std::abs(est - target) < 0.05);

    printf("test_model_accuracy PASSED\n\n");
}

void test_full_flow() {
    printf("Running test_full_flow (Build Model -> Charge)...\n");
    reset_globals();
    currentAppState = APP_STATE_BUILDING_MODEL;
    buildModelPhase = BuildModelPhase::Idle;

    int loop_count = 0;
    while (loop_count++ < 200000) {
        mock_millis += 100;
        sim.update(0.1f, dutyCycle);
        systemData.update();

        if (currentAppState == APP_STATE_BUILDING_MODEL) {
            buildCurrentModelStep();
        } else if (currentAppState == APP_STATE_CHARGING) {
            if (mock_millis - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = mock_millis;
                chargeBattery();
            }
        }

        if (currentAppState == APP_STATE_CHARGING && chargingState == CHARGE_MONITOR && mAh_charged > 100) break;
    }

    assert(currentAppState == APP_STATE_CHARGING);
    assert(chargingState == CHARGE_MONITOR);
    assert(mAh_charged > 100);
    printf("Successfully transitioned from model building to active charging.\n");
    printf("test_full_flow PASSED\n\n");
}

int main() {
    test_model_accuracy();
    test_full_flow();
    printf("ALL TESTS PASSED!\n");
    return 0;
}
