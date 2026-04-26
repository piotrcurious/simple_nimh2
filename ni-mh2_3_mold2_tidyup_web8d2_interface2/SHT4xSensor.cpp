#include "SHT4xSensor.h"

SHT4xSensor::SHT4xSensor() : _temperature(0), _humidity(0) {}

#ifndef MOCK_TEST
bool SHT4xSensor::begin() {
    if (!_sht4.begin(&Wire)) {
        return false;
    }
    _sht4.setPrecision(SHT4X_HIGH_PRECISION);
    _sht4.setHeater(SHT4X_NO_HEATER);
    return true;
}

void SHT4xSensor::read() {
    sensors_event_t humidity, temp;
    _sht4.getEvent(&humidity, &temp);
    _temperature = temp.temperature;
    _humidity = humidity.relative_humidity;
}

void SHT4xSensor::setPrecision(sht4x_precision_t precision) {
    _sht4.setPrecision(precision);
}

void SHT4xSensor::setHeater(sht4x_heater_t heater) {
    _sht4.setHeater(heater);
}
#endif
