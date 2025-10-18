#include "Sensors.h"

extern volatile float voltage_mv;
extern volatile float current_ma;

Sensors::Sensors(DataStore& dataStore) :
    data(dataStore),
    sht4xSensor(),
    thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, 0.0)
{}

void Sensors::begin() {
    sht4xSensor.begin();
    thermistorSensor.begin();
}

void Sensors::read() {
    sht4xSensor.read();
    while (sht4xSensor.isLocked()) {
        yield();
    }
    thermistorSensor.read(sht4xSensor.getTemperature());

    // Update the data store with the latest sensor readings
    data.updateHistory(sht4xSensor.getTemperature(), thermistorSensor.getTemperature2(), thermistorSensor.getDifference(), voltage_mv / 1000.0f, current_ma / 1000.0f);
}