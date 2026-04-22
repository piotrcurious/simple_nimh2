#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <stdarg.h>

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
float maximumCurrent = 0.150f;
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

    void update(float dt_s, int duty) {
        float current = (duty / 255.0f) * 0.5f;
        soc += (current * dt_s) / 3600.0f;
        voltage = 1.1f + 0.3f * soc + current * 0.2f;

        float P_heat = current * current * 0.2f + current * 0.05f;
        float dT = (P_heat * dt_s) / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT);
        float cooling = (temp - ambient) * DEFAULT_CONVECTIVE_H * dt_s;
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
    current = (dutyCycle / 255.0f) * 0.5f;
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
}

float estimateCurrent(int duty) { return (duty / 255.0f) * 0.5f; }
int estimateDutyCycleForCurrent(float target) { return (int)((target / 0.5f) * 255.0f); }

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
bool performLinearRegression(float data[][2], int count, float& s, float& i) { return false; }

int main() {
    printf("Starting Ni-MH Web Interface Mock Test...\n");

    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_IDLE;

    for (int i = 0; i < 3600; i++) {
        mock_millis += 1000;
        sim.update(1.0f, dutyCycle);

        float current_a = (dutyCycle / 255.0f) * 0.5f;
        mAh_charged += (current_a * 1000.0) / 3600.0;

        chargeBattery();

        if (i % 600 == 0) {
            printf("Time: %4ds | V: %.3fV | I: %.3fA | T: %.2fC | State: %d\n", i, sim.voltage, current_a, sim.temp, chargingState);
        }

        if (chargingState == CHARGE_STOPPED) {
            printf("Charging STOPPED at %ds due to safety threshold.\n", i);
            break;
        }
    }

    printf("Mock test finished. Log entries: %lu\n", (unsigned long)chargeLog.size());
    return 0;
}
