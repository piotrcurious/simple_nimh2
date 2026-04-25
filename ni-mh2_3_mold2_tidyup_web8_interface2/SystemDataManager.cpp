#include "SystemDataManager.h"
#include "adc_dma.h"
#include <cmath>
#include <cstring>
#include "definitions.h"

/**
 * DATA PATH SPEEDS:
 * 1. ADC RAW SAMPLING: 24,000 Hz (Continuous DMA)
 * 2. DMA FRAME PROCESSING: 100 Hz (task_processAdcDma)
 *    - Transfers batches of ~25 samples per channel to ring buffer.
 * 3. PHYSICAL DATA CONVERSION: 20 Hz (SystemDataManager::update)
 *    - CURRENT: Updated every 50ms (20Hz). Averages ~1,200 raw samples per update.
 *    - VOLTAGE: Updated every 250ms (4Hz). Averages ~6,000 raw samples per update.
 *    - TEMPERATURE (Battery): Updated every 50ms (20Hz) with 0.1 EMA smoothing.
 *      Averages ~1,200 raw samples per update. Inter-dependent on VCC and Ambient Temp.
 *    - TEMPERATURE (Ambient): Updated every 100ms (10Hz) via SHT4x I2C task.
 */

// External estimateCurrent for mAh calculation if below threshold
extern float estimateCurrent(int duty);
extern uint32_t dutyCycle;
extern CurrentModel currentModel;

SystemDataManager::SystemDataManager(SHT4xSensor& sht4, int therm1Pin, int vccPin, double therm1Offset)
    : _sht4(sht4), _therm1Pin(therm1Pin), _vccPin(vccPin), _therm1Offset(therm1Offset),
      _lastVoltageUpdateMs(0), _lastMahUpdateMs(0), _currentZeroOffsetMv(CURRENT_SHUNT_PIN_ZERO_OFFSET) {
    _dataMutex = nullptr;
    memset(&_currentData, 0, sizeof(_currentData));
    _currentData.current_sample_count = 1; // Non-zero start
    memset(_lastSnapshots, 0, sizeof(_lastSnapshots));

    // Initial reasonable values
    _currentData.battery_voltage_v = 1.2f;
    _currentData.ambient_temp_c = 25.0;
    _currentData.battery_temp_c = 25.0;
    _currentData.vcc_mv = 3300.0f;
}

void SystemDataManager::begin() {
    if (_dataMutex == nullptr) {
        _dataMutex = xSemaphoreCreateMutex();
        if (_dataMutex == nullptr) {
            Serial.println("SystemDataManager: FAILED to create mutex!");
        }
    }
    _lastMahUpdateMs = millis();
    Serial.println("SystemDataManager: Initialized.");
}

void SystemDataManager::update() {
    uint32_t now = millis();

    // 1. Get Ambient Temp from SHT4x (already being updated by its own task)
    double ambientTemp = _sht4.getTemperature();

    // 2. Process ADC DMA Snapshots for Current, Voltage, VCC, Therm1
    processAdcSnapshots();

    // 3. Perform inter-related calculations
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _currentData.ambient_temp_c = ambientTemp;
        _currentData.temp_diff_c = _currentData.battery_temp_c - _currentData.ambient_temp_c;

        // 4. Integrated mAh calculation (Centralized)
        uint32_t delta_ms = now - _lastMahUpdateMs;
        if (delta_ms > 0) {
            double delta_h = (double)delta_ms / 3600000.0;
            float current_ma = 0.0f;

            if (dutyCycle > 0) {
                current_ma = _currentData.charge_current_a * 1000.0f;

                // Handle low current estimation if model is available
                if (currentModel.isModelBuilt && _currentData.charge_current_a < MEASURABLE_CURRENT_THRESHOLD) {
                    current_ma = estimateCurrent(dutyCycle) * 1000.0f;
                }
            }

            _currentData.mah_charged += current_ma * delta_h;
            _lastMahUpdateMs = now;
        }

        _currentData.last_update_ms = now;
        xSemaphoreGive(_dataMutex);
    }
}

void SystemDataManager::processAdcSnapshots() {
    uint32_t now = millis();

    // We take snapshots for all relevant channels
    AdcSnapshot currentSnap, voltageSnap, vccSnap, therm1Snap;
    getAdcSnapshot(ADC_IDX_CURRENT, currentSnap);
    getAdcSnapshot(ADC_IDX_VOLTAGE, voltageSnap);
    getAdcSnapshot(ADC_IDX_THERM_VCC, vccSnap);
    getAdcSnapshot(ADC_IDX_THERM1, therm1Snap);

    // CURRENT (sampled at full rate, averaged since last update)
    if (currentSnap.count != _lastSnapshots[ADC_IDX_CURRENT].count) {
        uint32_t avgRawCurrent = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_CURRENT], currentSnap);
        float mv = snapshotToMillivolts(ADC_IDX_CURRENT, avgRawCurrent);
        float shuntMv = mv - _currentZeroOffsetMv;
        float currentA = (shuntMv / (float)CURRENT_SHUNT_RESISTANCE) / 1000.0f;

        if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _currentData.charge_current_a = std::max(0.0f, currentA);
            _currentData.current_mv = mv;
            _currentData.current_sample_count++;
            xSemaphoreGive(_dataMutex);
        }
        _lastSnapshots[ADC_IDX_CURRENT] = currentSnap;
    }

    // VCC (needed for voltage and thermistor)
    if (vccSnap.count != _lastSnapshots[ADC_IDX_THERM_VCC].count) {
        uint32_t avgRawVcc = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_THERM_VCC], vccSnap);
        float vccMv = snapshotToMillivolts(ADC_IDX_THERM_VCC, avgRawVcc);
        if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _currentData.vcc_mv = vccMv;
            xSemaphoreGive(_dataMutex);
        }
        _lastSnapshots[ADC_IDX_THERM_VCC] = vccSnap;
    }

    // THERMISTOR 1 -> Battery Temp
    if (therm1Snap.count != _lastSnapshots[ADC_IDX_THERM1].count) {
        uint32_t avgRawTherm1 = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_THERM1], therm1Snap);
        float therm1Mv = snapshotToMillivolts(ADC_IDX_THERM1, avgRawTherm1);
        double battTemp = calculateBatteryTemp(_currentData.ambient_temp_c, therm1Mv, _currentData.vcc_mv);

        if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Apply slight smoothing as in original code
            _currentData.battery_temp_c = 0.9 * _currentData.battery_temp_c + 0.1 * battTemp;
            xSemaphoreGive(_dataMutex);
        }
        _lastSnapshots[ADC_IDX_THERM1] = therm1Snap;
    }

    // VOLTAGE (Throttled update, e.g., 4Hz)
    if (now - _lastVoltageUpdateMs >= 250) {
        if (voltageSnap.count != _lastSnapshots[ADC_IDX_VOLTAGE].count) {
            uint32_t avgRawVoltage = calculateSnapshotAverage(_lastSnapshots[ADC_IDX_VOLTAGE], voltageSnap);
            float sampledMv = snapshotToMillivolts(ADC_IDX_VOLTAGE, avgRawVoltage);
            float batteryV = ((_currentData.vcc_mv * (float)MAIN_VCC_RATIO) - sampledMv) / 1000.0f;

            if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                _currentData.battery_voltage_v = batteryV;
                xSemaphoreGive(_dataMutex);
            }
            _lastSnapshots[ADC_IDX_VOLTAGE] = voltageSnap;
            _lastVoltageUpdateMs = now;
        }
    }
}

double SystemDataManager::calculateBatteryTemp(double ambientTemp, float therm1Mv, float vccMv) {
    double averageAnalogValue = (double)therm1Mv - _therm1Offset;
    double vcc_millivolts = (double)vccMv;

    double vRatio = averageAnalogValue / ((vcc_millivolts * MAIN_VCC_RATIO) - averageAnalogValue);
    if (vRatio <= 0) return ambientTemp; // Avoid log of non-positive

    double logVRatio = log(vRatio);
    double ambientKelvin = ambientTemp + 273.15;

    double invBattKelvin = (1.0 / ambientKelvin) + (logVRatio / BCOEFFICIENT);
    double battKelvin = 1.0 / invBattKelvin;
    return battKelvin - 273.15;
}

SystemData SystemDataManager::getData() {
    SystemData d;
    if (_dataMutex && xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        d = _currentData;
        xSemaphoreGive(_dataMutex);
    } else {
        // Fallback or debug logging
        memset(&d, 0, sizeof(d));
        if (_dataMutex) Serial.println("SystemDataManager::getData: Mutex timeout!");
    }
    return d;
}

void SystemDataManager::resetMah() {
    if (xSemaphoreTake(_dataMutex, portMAX_DELAY) == pdTRUE) {
        _currentData.mah_charged = 0;
        xSemaphoreGive(_dataMutex);
    }
}

void SystemDataManager::setCurrentZeroOffsetMv(float mv) {
    if (xSemaphoreTake(_dataMutex, portMAX_DELAY) == pdTRUE) {
        _currentZeroOffsetMv = mv;
        xSemaphoreGive(_dataMutex);
    }
}

float SystemDataManager::getCurrentZeroOffsetMv() {
    float mv = 0;
    if (xSemaphoreTake(_dataMutex, portMAX_DELAY) == pdTRUE) {
        mv = _currentZeroOffsetMv;
        xSemaphoreGive(_dataMutex);
    }
    return mv;
}
