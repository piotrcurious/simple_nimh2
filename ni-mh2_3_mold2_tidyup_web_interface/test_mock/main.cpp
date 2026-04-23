#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>

#include "dummy_esp32.h"

// Math helpers
using std::isnan;
using std::isfinite;
using std::min;
using std::max;

MockSerial Serial;
unsigned long mock_millis = 0;

#define MOCK_TEST
#define ArduinoEigenDense_h
namespace Eigen {
    struct VectorXd {
        int size() const {return 1;}
        float operator()(int i) const {return 0;}
        float& operator()(int i) { static float dummy; return dummy; }
    };
    struct MatrixXd { MatrixXd(int r, int c){} };
    struct QR { VectorXd solve(VectorXd b) const { return b; } };
    struct HouseholderQR { QR householderQr() const { return QR(); } };
}
namespace ArduinoEigen { using namespace Eigen; }

// Redirect definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define WebServer_h
#define Adafruit_SHT4x_h
#define ADC_CONTINUOUS_H
#define ESP_ADC_CAL_H
#define ADC_H

#include "../ni-mh2_3_mold2_tidyup_web_interface/definitions.h"

// Define externs
float maximumCurrent = 0.500f; // Set to something reasonable for testing
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;
uint32_t dutyCycle = 0;
bool isCharging = false;
bool isMeasuringResistance = false;
ChargingState chargingState = CHARGE_IDLE;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;
unsigned long chargePhaseStartTime = 0;
unsigned long chargingStartTime = 0;
uint8_t overtemp_trip_counter = 0;
unsigned long lastChargeEvaluationTime = 0;
const int pwmPin = 19;
double THERMISTOR_1_OFFSET = 0.0;
AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;

float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];
float voltage_values[PLOT_WIDTH];
float current_values[PLOT_WIDTH];

CurrentModel currentModel;
AsyncMeasure meas;
FindOptManager findOpt;
RemeasureManager remeasure;
std::vector<ChargeLogData> chargeLog;

float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0;
float regressedInternalResistanceIntercept = 0.2f;
float regressedInternalResistancePairsSlope = 0;
float regressedInternalResistancePairsIntercept = 0.18f;

unsigned long lastPlotUpdateTime = 0;
unsigned long lastChargingHouseTime = 0;

// Mock hardware functions
void applyDuty(uint32_t duty) { dutyCycle = duty; }
void analogWrite(int pin, int val) { (void)pin; (void)val; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }

// Implementation mocks
void logChargeData(const ChargeLogData& data) {
    chargeLog.push_back(data);
}

// SHT4xSensor dummy impl
bool SHT4xSensor::begin() { return true; }
void SHT4xSensor::read() {}
void SHT4xSensor::setPrecision(sht4x_precision_t p) {}
void SHT4xSensor::setHeater(sht4x_heater_t h) {}

// SystemDataManager dummy impl
SystemDataManager::SystemDataManager(SHT4xSensor& s, int p1, int p2, double o) : _sht4(s) {}
void SystemDataManager::begin() {}
void SystemDataManager::update() {}
void SystemDataManager::resetMah() { mAh_charged = 0; }
SystemData SystemDataManager::getData() {
    return { 22.0f, 24.0f, 2.0f, (float)voltage_mv/1000.0f, (float)current_ma/1000.0f, (float)mAh_charged };
}

// Physics Simulation
struct BatterySim {
    float voltage = 1.2f;
    float temp = 22.0f;
    float ambient = 22.0f;
    float capacity_ah = 2.0f;
    float soc = 0.1f;
    float internal_resistance = 0.2f;

    void update(float dt_s, int duty) {
        float current = (duty / 255.0f) * 1.0f; // Max 1A
        soc += (current * dt_s) / 3600.0f;
        voltage = 1.1f + 0.3f * soc + current * internal_resistance;

        // Thermal: P = I^2 * R + charging chemistry heat (exothermic)
        float P_heat = current * current * internal_resistance + current * 0.1f;
        float dT = (P_heat * dt_s) / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT);

        // Newton's law of cooling
        float cooling = (temp - ambient) * 0.1f * dt_s; // Arbitrary cooling factor
        temp += dT - cooling;
    }
};

BatterySim sim;

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    temp1 = sim.ambient;
    temp2 = sim.temp;
    tempDiff = temp2 - temp1;
    t1_millivolts = 0;
    voltage = sim.voltage;
    current = (dutyCycle / 255.0f) * 1.0f;
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
}

float estimateCurrent(int duty) { return (duty / 255.0f) * 1.0f; }
int estimateDutyCycleForCurrent(float target) { return (int)((target / 1.0f) * 255.0f); }

float eval_mAh_snapshot = 0.0f;
unsigned long eval_time_snapshot = 0;
bool reeval_active = false;
float reeval_start_mAh = 0.0f;
unsigned long reeval_start_ms = 0;
float lastReeval_delta_mAh = 0.0f;
unsigned long lastReeval_duration_ms = 0;
float lastReeval_avgCurrent_A = 0.0f;
float currentRampTarget = 0.0f;

#include "../ni-mh2_3_mold2_tidyup_web_interface/charging.cpp"

// Define missing symbols from internal_resistance.cpp if not including it
IRState currentIRState = IR_STATE_IDLE;
void measureInternalResistanceStep() {}
void storeOrAverageResistanceData(float c, float r, float data[][2], int& count) {}
void bubbleSort(float data[][2], int n) {}
void distribute_error(float data[][2], int count, float s, float e) {}
bool performLinearRegression(float data[][2], int count, float& s, float& i) {
    if (count < 2) return false;
    s = 0; i = 0.2f; return true;
}

void reset_globals() {
    mock_millis = 0;
    voltage_mv = 1000.0f;
    current_ma = 0.0f;
    mAh_charged = 0.0;
    dutyCycle = 0;
    chargingState = CHARGE_IDLE;
    chargeLog.clear();
    sim = BatterySim();
    overtemp_trip_counter = 0;
    currentAppState = APP_STATE_IDLE;
    currentRampTarget = 0.0f;
    eval_mAh_snapshot = 0.0f;
    eval_time_snapshot = 0;
}

void test_overtemp_shutdown() {
    printf("Running test_overtemp_shutdown...\n");
    reset_globals();
    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_IDLE;

    // Artificially increase battery temp to trigger overtemp
    sim.temp = 60.0f;
    sim.ambient = 20.0f;
    chargingState = CHARGE_MONITOR; // skip find_opt for fast test

    bool shutDown = false;
    for (int i = 0; i < 7200; i++) {
        mock_millis += 1000;
        sim.update(1.0f, dutyCycle);

        // Force the temperature to stay high
        sim.temp = 60.0f;

        // Trigger evaluation faster
        if (i % 240 == 0) eval_time_snapshot = mock_millis - CHARGE_EVALUATION_INTERVAL_MS - 1;

        if (!chargeBattery()) {
            shutDown = true;
            printf("Confirmed: Shutdown triggered at t=%ds\n", i);
            break;
        }
    }
    assert(shutDown);
    assert(chargingState == CHARGE_STOPPED);
    printf("test_overtemp_shutdown PASSED\n\n");
}

void test_thermal_model_unit() {
    printf("Running test_thermal_model_unit...\n");
    // Test the estimateTempDiff function directly
    float unapplied = 0.0f;
    float diff = estimateTempDiff(1.2, 1.2, 0.5, 0.2, 25.0, 1000, 0, 25.0, &unapplied);
    printf("Estimated temp diff after 1s at 0.5A: %.4f\n", diff);
    assert(diff > 0);

    // No current, no temp change
    diff = estimateTempDiff(1.2, 1.2, 0.0, 0.2, 25.0, 2000, 1000, 25.0, &unapplied);
    printf("Estimated temp diff after 1s at 0.0A: %.4f\n", diff);
    assert(std::abs(diff) < 0.001);

    printf("test_thermal_model_unit PASSED\n\n");
}

void test_charging_ramp() {
    printf("Running test_charging_ramp...\n");
    reset_globals();
    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_IDLE;
    maximumCurrent = 0.5f;

    float lastCurrent = 0;
    int rampUpCount = 0;

    for (int i = 0; i < 7200; i++) { // 2 hours
        mock_millis += 1000;
        sim.update(1.0f, dutyCycle);

        float current_a = (dutyCycle / 255.0f) * 1.0f;
        mAh_charged += (current_a * 1000.0) / 3600.0;

        chargeBattery();

        if (i > 0 && i % 240 == 0) { // Every evaluation interval
            if (currentRampTarget > lastCurrent) {
                rampUpCount++;
            }
            lastCurrent = currentRampTarget;
        }
    }
    printf("Final Ramp Target: %.3fA, Ramp Ups: %d\n", currentRampTarget, rampUpCount);
    assert(currentRampTarget > 0);
    assert(rampUpCount > 0);
    printf("test_charging_ramp PASSED\n\n");
}

int main() {
    test_thermal_model_unit();
    test_overtemp_shutdown();
    test_charging_ramp();

    printf("ALL TESTS PASSED!\n");
    return 0;
}
