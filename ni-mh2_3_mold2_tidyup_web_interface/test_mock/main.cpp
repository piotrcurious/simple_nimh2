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

// Global variable definitions
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

// Hardware function stubs
void applyDuty(uint32_t duty) { dutyCycle = duty; }

// Physics Simulation
struct BatterySim {
    float voltage = 1.15f;
    float temp = 22.0f;
    float ambient = 22.0f;
    float soc = 0.05f; // Almost empty
    float internal_resistance = 0.15f;

    void update(float dt_s, int duty) {
        float current = (duty / 255.0f) * 2.0f; // Max 2A
        soc += (current * dt_s) / 7200.0f; // 2000mAh battery (7200 Coulombs)

        // Simple NiMH voltage curve + IR drop
        float base_v = 1.1f + 0.3f * soc;
        if (soc > 0.9f) base_v += (soc - 0.9f) * 2.0f; // Rapid rise at end

        voltage = base_v + current * internal_resistance;

        // Thermal: Overcharge heat
        float overcharge_P = 0;
        if (soc > 1.0f) overcharge_P = current * 1.4f; // All energy to heat

        float P_heat = current * current * internal_resistance + current * 0.1f + overcharge_P;
        float dT = (P_heat * dt_s) / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT);
        float cooling = (temp - ambient) * 0.15f * dt_s;
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
    current = (dutyCycle / 255.0f) * 2.0f;
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
}

float estimateCurrent(int duty) { return (duty / 255.0f) * 2.0f; }
int estimateDutyCycleForCurrent(float target) { return (int)((target / 2.0f) * 255.0f); }

// SHT4xSensor dummy impl
SHT4xSensor::SHT4xSensor() : _temperature(22.0f), _humidity(50.0f) {}
bool SHT4xSensor::begin() { return true; }
void SHT4xSensor::read() {}
void SHT4xSensor::setPrecision(sht4x_precision_t p) {}
void SHT4xSensor::setHeater(sht4x_heater_t h) {}

// SystemDataManager dummy impl
SystemDataManager::SystemDataManager(SHT4xSensor& s, int p1, int p2, double o) : _sht4(s) {
    _dataMutex = xSemaphoreCreateMutex();
}
void SystemDataManager::begin() {}
void SystemDataManager::update() {
    static uint32_t last_m = 0;
    uint32_t now = mock_millis;
    if (last_m == 0) last_m = now;
    float dt_h = (now - last_m) / 3600000.0f;
    mAh_charged += (current_ma * dt_h);
    last_m = now;
}
void SystemDataManager::resetMah() { mAh_charged = 0; }
SystemData SystemDataManager::getData() {
    return { (float)voltage_mv/1000.0f, (float)current_ma/1000.0f, 22.0, (double)sim.temp, (double)(sim.temp-22.0), 5000.0f, (float)mAh_charged, (uint32_t)mock_millis };
}

SHT4xSensor sht4Sensor;
SystemDataManager systemData(sht4Sensor, 36, 35, 0.0);
CurrentModel currentModel;

#include "../ni-mh2_3_mold2_tidyup_web_interface/home_screen.h"
HomeScreen::HomeScreen() {}
void HomeScreen::begin() {}
void HomeScreen::gatherData() {}
HomeScreen homeScreen;

// Include all .cpp files to get definitions
#include "../ni-mh2_3_mold2_tidyup_web_interface/charging.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/internal_resistance.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/graphing.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/logging.cpp"
#include "../ni-mh2_3_mold2_tidyup_web_interface/web_handlers.cpp"

WebServer server;

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
    currentIRState = IR_STATE_IDLE;
    isMeasuringResistance = false;
    isCharging = false;
    recentChargeLogsCount = 0;
    recentChargeLogsHead = 0;
}

void test_ir_state_machine() {
    printf("Running test_ir_state_machine...\n");
    reset_globals();
    isMeasuringResistance = true;
    currentIRState = IR_STATE_START;
    for (int i = 0; i < 200000; i++) {
        mock_millis += 10;
        sim.update(0.01f, dutyCycle);
        measureInternalResistanceStep();
        if (currentIRState == IR_STATE_IDLE) break;
        if (currentIRState == IR_STATE_GET_MEASUREMENT || currentIRState == IR_STATE_STOP_LOAD_WAIT) mock_millis += 1000;
    }
    printf("Final IR State: %d, Data Count: %d, Pairs: %d\n", currentIRState, resistanceDataCount, resistanceDataCountPairs);
    assert(currentIRState == IR_STATE_IDLE);
    assert(resistanceDataCount > 0);
    printf("test_ir_state_machine PASSED\n\n");
}

void test_full_charge_simulation() {
    printf("Running test_full_charge_simulation...\n");
    reset_globals();
    maximumCurrent = 0.8f; // 800mA
    startCharging();

    // Simulate 5 hours (18000 seconds)
    const float dt = 1.0f; // 1s steps
    for (int t = 0; t < 18000; t++) {
        mock_millis += (uint32_t)(dt * 1000);
        sim.update(dt, dutyCycle);
        systemData.update();

        bool active = chargeBattery();

        if (t % 600 == 0) {
            printf("  t=%4ds: State=%d, SOC=%.2f, V=%.3f, I=%.3f, mAh=%.1f, Temp=%.1f\n",
                   t, chargingState, sim.soc, sim.voltage, (float)current_ma/1000.0f, (float)mAh_charged, sim.temp);
        }

        if (!active) {
            printf("Charging finished at t=%ds. Reason: %d, mAh=%.1f\n", t, chargingState, (float)mAh_charged);
            break;
        }
    }
    assert(mAh_charged > 500);
    printf("test_full_charge_simulation PASSED\n\n");
}

int main() {
    test_ir_state_machine();
    test_full_charge_simulation();
    printf("ALL MOCK TESTS PASSED!\n");
    return 0;
}
