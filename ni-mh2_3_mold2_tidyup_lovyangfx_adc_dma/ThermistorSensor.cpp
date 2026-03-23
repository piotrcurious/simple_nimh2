// ThermistorSensor.cpp
#include "ThermistorSensor.h"
#include <Arduino.h>
#include "analog.h"
#include "adc_dma.h"

ThermistorSensor::ThermistorSensor(int thermistor1Pin, int thermistorVccPin, double thermistor1Offset)
    : thermistor1Pin(thermistor1Pin), thermistorVccPin(thermistorVccPin), thermistor1Offset(thermistor1Offset),
      vcc_millivolts(1000.0), thermistor1Value(25.0), thermistor2Value(25.0), thermistorDiffValue(0.0),
      thermistorVccValue(600.0), thermistor1RawMillivolts(300.0), lock(false), last_time(0) {
    last_snapshot_vcc = {0, 0};
    last_snapshot_therm1 = {0, 0};
}

void ThermistorSensor::begin() {
}

void ThermistorSensor::read(double topThermistorTemperatureCelsius) {
    uint32_t current_time = millis();
    if ((current_time - last_time) > update_interval) {
        lock = true;

        double currentVCC = readVCCInternal(thermistorVccPin);
        if (!std::isnan(currentVCC)) {
            thermistorVccValue = currentVCC;
            vcc_millivolts = thermistorVccValue;
        }

        // Use snapshot to read Thermistor 1 raw millivolts
        AdcSnapshot new_snapshot_therm1;
        getAdcSnapshot(ADC_IDX_THERM1, new_snapshot_therm1);
        uint32_t avg_raw1 = calculateSnapshotAverage(last_snapshot_therm1, new_snapshot_therm1);
        float rawMillivolts1 = 0;
        if (avg_raw1 > 0) {
            rawMillivolts1 = snapshotToMillivolts(ADC_IDX_THERM1, avg_raw1);
            last_snapshot_therm1 = new_snapshot_therm1;
        } else {
            // Fallback if no new samples
            rawMillivolts1 = getAdcMillivolts(ADC_IDX_THERM1);
        }

        // Calculate derived values
        double temp2 = readThermistorRelativeInternal(thermistor1Pin, topThermistorTemperatureCelsius, thermistor1Offset);
        double tempDiff = -topThermistorTemperatureCelsius + temp2;

        const float alpha = 0.1;
        thermistor1Value = (1.0 - alpha) * thermistor1Value + alpha * topThermistorTemperatureCelsius;
        thermistor2Value = (1.0 - alpha) * thermistor2Value + alpha * temp2;
        thermistorDiffValue = (1.0 - alpha) * thermistorDiffValue + alpha * tempDiff;
        thermistor1RawMillivolts = (1.0 - alpha) * thermistor1RawMillivolts + alpha * rawMillivolts1;

        lock = false;
        last_time = current_time;
    }
}

double ThermistorSensor::getTemperature1() {
    return thermistor1Value;
}

double ThermistorSensor::getTemperature2() {
    return thermistor2Value;
}

double ThermistorSensor::getDifference() {
    return thermistorDiffValue;
}

double ThermistorSensor::getVCC() {
    return thermistorVccValue;
}

float ThermistorSensor::getRawMillivolts1() {
    return thermistor1RawMillivolts;
}

bool ThermistorSensor::isLocked() {
    return lock;
}

double ThermistorSensor::readVCCInternal(int pin) {
    AdcSnapshot new_snapshot_vcc;
    getAdcSnapshot(ADC_IDX_THERM_VCC, new_snapshot_vcc);
    uint32_t avg_raw = calculateSnapshotAverage(last_snapshot_vcc, new_snapshot_vcc);

    if (avg_raw == 0) {
        // Fallback to legacy behavior if no new samples
        return getAdcMillivolts(ADC_IDX_THERM_VCC);
    }

    last_snapshot_vcc = new_snapshot_vcc;
    return (double)snapshotToMillivolts(ADC_IDX_THERM_VCC, avg_raw);
}

double ThermistorSensor::readThermistorRelativeInternal(
    int pin, double topThermistorTemperatureCelsius, double offset) {

    // Thermistor 1 raw millivolts already averaged during 'read' call
    // but here we might be using snapshot for precise instantaneous value
    AdcSnapshot new_s;
    getAdcSnapshot(ADC_IDX_THERM1, new_s);
    uint32_t avg_raw = calculateSnapshotAverage(last_snapshot_therm1, new_s);
    float current_mv = 0;
    if (avg_raw > 0) {
        current_mv = snapshotToMillivolts(ADC_IDX_THERM1, avg_raw);
        // Do NOT update last_snapshot_therm1 here as it is used in the main read() loop
    } else {
        current_mv = getAdcMillivolts(ADC_IDX_THERM1);
    }

    double averageAnalogValue = (double)current_mv - offset;

    double vRatio = averageAnalogValue / ((vcc_millivolts * MAIN_VCC_RATIO) - averageAnalogValue);
    double logVRatio = log(vRatio);

    double topThermistorTemperatureKelvin = topThermistorTemperatureCelsius + 273.15;
    double beta = BCOEFFICIENT;

    double invBottomTempKelvin = (1.0 / topThermistorTemperatureKelvin) + (logVRatio / beta);
    double bottomThermistorTemperatureKelvin = 1.0 / invBottomTempKelvin;
    double bottomThermistorTemperatureCelsius = bottomThermistorTemperatureKelvin - 273.15;

    return bottomThermistorTemperatureCelsius;
}

double ThermistorSensor::resistanceToTemperature(double resistance) {
    double steinhart;
    steinhart = log(resistance / THERMISTORNOMINAL);
    steinhart /= BCOEFFICIENT;
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    return steinhart - 273.15;
}

double ThermistorSensor::temperatureToResistance(double temperatureCelsius) {
    double temperatureKelvin = temperatureCelsius + 273.15;
    return THERMISTORNOMINAL * exp(BCOEFFICIENT * (1.0 / temperatureKelvin - 1.0 / (TEMPERATURENOMINAL + 273.15)));
}

float ThermistorSensor::mapfInternal(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
