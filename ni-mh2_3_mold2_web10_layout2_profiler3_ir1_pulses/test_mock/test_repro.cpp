#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>
#include <cstdlib>

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

#include "../adc_dma.h"

// We need to include SystemDataManager.h with private members accessible
#define private public
#include "../SystemDataManager.h"
#undef private

#include "../definitions.h"
#include "../internal_resistance.h"
#include "../AdvancedPolynomialFitter.hpp"

// Hardware stubs that would normally be in .ino
void applyDuty(uint32_t duty);
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
float estimateCurrent(int duty);
int estimateDutyCycleForCurrent(float target);

// Physics Simulation
struct BatterySim {
    float voltage = 1.15f;
    float unloaded_voltage = 1.25f;
    float temp = 22.0f;
    float ambient = 22.0f;
    float soc = 0.05f;
    float internal_resistance = 0.15f;
    float capacity_ah = 2.0f;
    float noise_mv = 0.0f;

    // Non-linear duty-to-current mapping
    float getCurrent(int duty) {
        if (duty < 20) return 0.0f;
        float normalized = (float)(duty - 20) / 235.0f;
        return 2.5f * (normalized * normalized);
    }

    void update(float dt_s, int duty) {
        float current = getCurrent(duty);
        soc += (current * dt_s) / (capacity_ah * 3600.0f);
        float base_v = 1.2f + 0.15f * std::pow(soc, 0.5f);
        unloaded_voltage = base_v;
        voltage = base_v + current * internal_resistance;
    }
};
BatterySim sim;

// Global variables from .ino
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
float MEASURABLE_CURRENT_THRESHOLD = 0.005f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;
uint32_t dutyCycle = 0;
bool isCharging = false;
volatile AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;
unsigned long lastPlotUpdateTime = 0;
unsigned long lastChargingHouseTime = 0;
const int pwmPin = 19;
double THERMISTOR_1_OFFSET = 0.0;

void applyDuty(uint32_t duty) { dutyCycle = duty; }

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    SystemData d = systemData.getData();
    temp1 = d.ambient_temp_c;
    temp2 = d.battery_temp_c;
    tempDiff = d.temp_diff_c;
    t1_millivolts = 0;
    voltage = d.battery_voltage_v;
    current = d.charge_current_a;
    voltage_mv = voltage * 1000.0f;
    current_ma = current * 1000.0f;
}

// ADC DMA Stubs for SystemDataManager.cpp
static uint32_t mock_sample_count = 0;
void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot) {
    snapshot.count = mock_sample_count;
    float val_mv = 0;
    if (idx == ADC_IDX_CURRENT) {
        val_mv = sim.getCurrent((int)dutyCycle) * CURRENT_SHUNT_RESISTANCE * 1000.0f + systemData.getCurrentZeroOffsetMv() + sim.noise_mv;
    } else if (idx == ADC_IDX_VOLTAGE) {
        val_mv = (dutyCycle == 0 ? sim.unloaded_voltage : sim.voltage) * 1000.0f;
    } else if (idx == ADC_IDX_THERM1) {
        val_mv = 1500.0f;
    } else if (idx == ADC_IDX_THERM_VCC) {
        val_mv = 3300.0f;
    }
    snapshot.sum = (uint64_t)val_mv * snapshot.count;
}
uint32_t calculateSnapshotAverage(const AdcSnapshot &old_s, const AdcSnapshot &new_s) {
    uint32_t d_count = new_s.count - old_s.count;
    if (d_count == 0) return (uint32_t)(new_s.sum / (new_s.count ? new_s.count : 1));
    return (uint32_t)((new_s.sum - old_s.sum) / d_count);
}
float snapshotToMillivolts(AdcChannelIndex idx, uint32_t avg_raw) {
    return (float)avg_raw;
}
void setupAdcDma() {}
void processAdcDma() {}

SHT4xSensor::SHT4xSensor() : _temperature(0), _humidity(0) {}
bool SHT4xSensor::begin() { return true; }
void SHT4xSensor::read() { _temperature = sim.temp; _humidity = 50.0f; }

SystemDataManager::SystemDataManager(SHT4xSensor& sht4, int therm1Pin, int vccPin, double therm1Offset)
    : _sht4(sht4), _therm1Pin(therm1Pin), _vccPin(vccPin), _therm1Offset(therm1Offset), _currentZeroOffsetMv(75.0f) {
    _dataMutex = (SemaphoreHandle_t)1;
}
void SystemDataManager::begin() {}

// We will use the implementation from the actual file via include if possible,
// but here we are mocking SystemDataManager so we have to replicate the logic we want to test.
void SystemDataManager::update() {
    static uint32_t last_m = 0;
    uint32_t now = mock_millis;
    if (last_m == 0) last_m = now;
    processAdcSnapshots();

    uint32_t delta_ms = now - last_m;
    if (delta_ms > 0) {
        double delta_h = (double)delta_ms / 3600000.0;
        float current_ma = _currentData.charge_current_a * 1000.0f;

        float estimated_current_a = estimateCurrent(dutyCycle);
        if (currentModel.isModelBuilt && estimated_current_a < MEASURABLE_CURRENT_THRESHOLD) {
            current_ma = estimated_current_a * 1000.0f;
        }

        _currentData.mah_charged += current_ma * delta_h;
        last_m = now;
    }
}

void SystemDataManager::processAdcSnapshots() {
    AdcSnapshot currentSnap;
    getAdcSnapshot(ADC_IDX_CURRENT, currentSnap);
    uint32_t avgRawCurrent = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_CURRENT], currentSnap);
    _lastSnapshots[ADC_IDX_CURRENT] = currentSnap;
    _currentData.current_mv = snapshotToMillivolts(ADC_IDX_CURRENT, avgRawCurrent);

    float currentA = (_currentData.current_mv - _currentZeroOffsetMv) / (CURRENT_SHUNT_RESISTANCE * 1000.0f);
    _currentData.charge_current_a = currentA; // Removed clamp
    _currentData.current_sample_count = currentSnap.count;

    getAdcSnapshot(ADC_IDX_VOLTAGE, currentSnap);
    uint32_t avgRawVoltage = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_VOLTAGE], currentSnap);
    _lastSnapshots[ADC_IDX_VOLTAGE] = currentSnap;
    _currentData.battery_voltage_v = snapshotToMillivolts(ADC_IDX_VOLTAGE, avgRawVoltage) / 1000.0f;
}
SystemData SystemDataManager::getData() { return _currentData; }
void SystemDataManager::resetMah() { _currentData.mah_charged = 0; }
void SystemDataManager::setCurrentZeroOffsetMv(float mv) { _currentZeroOffsetMv = mv; }
float SystemDataManager::getCurrentZeroOffsetMv() { return _currentZeroOffsetMv; }

SHT4xSensor sht4Sensor;
SystemDataManager systemData(sht4Sensor, 36, 35, 0.0);
CurrentModel currentModel;

float estimateCurrent(int duty) {
    if (!currentModel.isModelBuilt) return 0.0f;
    const float x = static_cast<float>(duty);
    double sum = 0.0;
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        sum += currentModel.coefficients(i) * std::pow(x, i);
    }
    return static_cast<float>(std::max(0.0, sum));
}

void test_repro() {
    std::cout << "Running test_repro with 140mV offset and noise..." << std::endl;
    mock_millis = 0;
    mock_sample_count = 1;
    systemData.resetMah();

    // Simulate model built
    currentModel.isModelBuilt = true;
    currentModel.coefficients.resize(4);
    currentModel.coefficients(0) = 0.0;
    currentModel.coefficients(1) = 0.001;
    currentModel.coefficients(2) = 0.0001;
    currentModel.coefficients(3) = 0.000001;

    // Proper calibration to 140mV
    systemData.setCurrentZeroOffsetMv(140.0f);

    MEASURABLE_CURRENT_THRESHOLD = 0.010f; // 10mA. Shunt 2.5 Ohm -> 25mV unmeasurable region

    applyDuty(0);

    std::cout << "Starting simulation with duty 0, 140mV offset, and random noise..." << std::endl;
    srand(42);
    for(int i=0; i<1000; ++i) {
        mock_millis += 100;
        mock_sample_count++;
        // Add random noise between -30 and +30 mV
        sim.noise_mv = (float)(rand() % 61 - 30);
        systemData.update();
    }

    SystemData d = systemData.getData();
    std::cout << "After 100 seconds: mAh_charged = " << d.mah_charged << std::endl;

    if (d.mah_charged > 0.0001) {
        std::cout << "REPRODUCED: mAh counter advanced when duty cycle was 0!" << std::endl;
    } else {
        std::cout << "FAILED TO REPRODUCE." << std::endl;
    }

    std::cout << "test_repro complete." << std::endl << std::endl;
}

int main() {
    test_repro();
    return 0;
}
