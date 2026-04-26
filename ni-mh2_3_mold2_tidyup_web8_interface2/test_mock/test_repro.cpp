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
    float getCurrent(int duty) {
        if (duty < 20) return 0.0f;
        float normalized = (float)(duty - 20) / 235.0f;
        return 2.5f * (normalized * normalized);
    }
};
BatterySim sim;

// Global variables from .ino
float MEASURABLE_CURRENT_THRESHOLD = 0.010f; // 10mA
uint32_t dutyCycle = 0;

void applyDuty(uint32_t duty) { dutyCycle = duty; }

// ADC DMA Stubs for SystemDataManager.cpp
static uint32_t mock_sample_count = 0;
static uint64_t mock_current_sum = 0;

void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot) {
    if (idx == ADC_IDX_CURRENT) {
        snapshot.count = mock_sample_count;
        snapshot.sum = mock_current_sum;
    } else {
        snapshot.count = mock_sample_count;
        snapshot.sum = (uint64_t)1200 * snapshot.count;
    }
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
void SHT4xSensor::read() { _temperature = 25.0f; _humidity = 50.0f; }

SystemDataManager::SystemDataManager(SHT4xSensor& sht4, int therm1Pin, int vccPin, double therm1Offset)
    : _sht4(sht4), _therm1Pin(therm1Pin), _vccPin(vccPin), _therm1Offset(therm1Offset), _currentZeroOffsetMv(75.0f) {
    _dataMutex = (SemaphoreHandle_t)1;
}
void SystemDataManager::begin() {}

void SystemDataManager_buggy_update(SystemDataManager* self, uint32_t& last_m) {
    uint32_t now = mock_millis;

    AdcSnapshot currentSnap;
    getAdcSnapshot(ADC_IDX_CURRENT, currentSnap);
    uint32_t avgRawCurrent = calculateSnapshotAverage(self->_lastSnapshots[ADC_IDX_CURRENT], currentSnap);
    self->_lastSnapshots[ADC_IDX_CURRENT] = currentSnap;
    self->_currentData.current_mv = snapshotToMillivolts(ADC_IDX_CURRENT, avgRawCurrent);
    float currentA = (self->_currentData.current_mv - self->_currentZeroOffsetMv) / (CURRENT_SHUNT_RESISTANCE * 1000.0f);
    self->_currentData.charge_current_a = std::max(0.0f, currentA); // BUG: clamping

    uint32_t delta_ms = now - last_m;
    if (delta_ms > 0) {
        double delta_h = (double)delta_ms / 3600000.0;
        float current_ma = self->_currentData.charge_current_a * 1000.0f;

        if (currentModel.isModelBuilt && self->_currentData.charge_current_a < MEASURABLE_CURRENT_THRESHOLD) {
            current_ma = estimateCurrent(dutyCycle) * 1000.0f;
        }

        self->_currentData.mah_charged += (double)current_ma * delta_h;
        last_m = now;
    }
}

void SystemDataManager_fixed_update(SystemDataManager* self, uint32_t& last_m) {
    uint32_t now = mock_millis;

    AdcSnapshot currentSnap;
    getAdcSnapshot(ADC_IDX_CURRENT, currentSnap);
    uint32_t avgRawCurrent = calculateSnapshotAverage(self->_lastSnapshots[ADC_IDX_CURRENT], currentSnap);
    self->_lastSnapshots[ADC_IDX_CURRENT] = currentSnap;
    self->_currentData.current_mv = snapshotToMillivolts(ADC_IDX_CURRENT, avgRawCurrent);
    float currentA = (self->_currentData.current_mv - self->_currentZeroOffsetMv) / (CURRENT_SHUNT_RESISTANCE * 1000.0f);
    self->_currentData.charge_current_a = currentA; // FIXED: no clamping

    uint32_t delta_ms = now - last_m;
    if (delta_ms > 0) {
        double delta_h = (double)delta_ms / 3600000.0;
        float current_ma = self->_currentData.charge_current_a * 1000.0f;

        float estimated_current_a = estimateCurrent(dutyCycle);
        if (currentModel.isModelBuilt && estimated_current_a < MEASURABLE_CURRENT_THRESHOLD) {
            current_ma = estimated_current_a * 1000.0f;
        }

        self->_currentData.mah_charged += (double)current_ma * delta_h;
        last_m = now;
    }
}

SystemData SystemDataManager::getData() { return _currentData; }
void SystemDataManager::resetMah() { _currentData.mah_charged = 0; }
void SystemDataManager::setCurrentZeroOffsetMv(float mv) { _currentZeroOffsetMv = mv; }

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

void test_comparison() {
    std::cout << "Running test_comparison: Buggy vs Fixed with 140mV offset and noise..." << std::endl;

    auto run_sim = [](bool fixed, int duty) {
        mock_millis = 0;
        uint32_t last_m = 0;
        mock_sample_count = 0;
        mock_current_sum = 0;
        systemData.resetMah();
        memset(systemData._lastSnapshots, 0, sizeof(systemData._lastSnapshots));

        currentModel.isModelBuilt = true;
        currentModel.coefficients.resize(4);
        currentModel.coefficients(0) = 0.0;
        currentModel.coefficients(1) = 0.0001;
        currentModel.coefficients(2) = 0.00001;
        currentModel.coefficients(3) = 0.000001;

        systemData.setCurrentZeroOffsetMv(140.0f);

        applyDuty(duty);
        srand(42);
        double sum_measured_a = 0;
        int iterations = 10000;
        for(int i=0; i<iterations; ++i) {
            mock_millis += 10;
            for(int j=0; j<12; ++j) {
                mock_sample_count++;
                float noise = (float)(rand() % 161 - 80); // +/- 80mV -> +/- 32mA
                float val_mv = sim.getCurrent(duty) * 2500.0f + 140.0f + noise;
                mock_current_sum += (uint64_t)val_mv;
            }
            if (fixed) SystemDataManager_fixed_update(&systemData, last_m);
            else SystemDataManager_buggy_update(&systemData, last_m);
            sum_measured_a += systemData.getData().charge_current_a;
        }
        std::cout << "  Duty: " << duty << ", Avg Measured: " << (sum_measured_a / iterations) << " A, mAh: " << systemData.getData().mah_charged << std::endl;
        return systemData.getData().mah_charged;
    };

    std::cout << "--- Idle Case (Duty 0) ---" << std::endl;
    double mah_buggy_idle = run_sim(false, 0);
    double mah_fixed_idle = run_sim(true, 0);

    std::cout << "--- Low Power Case (Duty 30, expected ~0.002A) ---" << std::endl;
    double mah_buggy_low = run_sim(false, 30);
    double mah_fixed_low = run_sim(true, 30);

    if (mah_buggy_idle > 0.001 && mah_fixed_idle < 0.00001) {
        std::cout << "\nSUCCESS: Fixed logic eliminated idle drift." << std::endl;
    } else {
        std::cout << "\nFAILURE: " << std::endl;
    }

    std::cout << "test_comparison complete." << std::endl << std::endl;
}

int main() {
    test_comparison();
    return 0;
}
