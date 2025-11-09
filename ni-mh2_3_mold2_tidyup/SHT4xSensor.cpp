// SHT4xSensor.cpp
#include "SHT4xSensor.h"
#include <Arduino.h>

SHT4xSensor::SHT4xSensor() : last_time(0), update_interval(500), lock(false), temperature(25.0), humidity(50.0) {}

bool SHT4xSensor::begin() {
    if (!sht4.begin()) {
        Serial.println("Couldn't find SHT4x");
        return false;
    }
    return true;
}

void SHT4xSensor::read() {
    uint32_t current_time = millis();
    if ((current_time - last_time) > update_interval) {
        sensors_event_t humidity_event, temp_event;
        lock = true; // set lock to prevent reads
        if (sht4.getEvent(&humidity_event, &temp_event)) {
            temperature = (11.0 * temperature + temp_event.temperature) / 12.0; // average
            humidity = humidity_event.relative_humidity;
        } else {
            Serial.println("Failed to read SHT4x sensor");
        }
        lock = false; // release lock
        last_time = current_time;
    }
}

float SHT4xSensor::getTemperature() {
    return temperature;
}

float SHT4xSensor::getHumidity() {
    return humidity;
}

void SHT4xSensor::setPrecision(sht4x_precision_t precision) {
    sht4.setPrecision(precision);
}

sht4x_precision_t SHT4xSensor::getPrecision() {
    return sht4.getPrecision();
}

void SHT4xSensor::setHeater(sht4x_heater_t heater) {
    sht4.setHeater(heater);
}

sht4x_heater_t SHT4xSensor::getHeater() {
    return sht4.getHeater();
}

bool SHT4xSensor::isLocked() {
    return lock;
}
