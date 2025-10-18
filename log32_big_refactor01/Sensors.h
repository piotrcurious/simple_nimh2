#ifndef SENSORS_H
#define SENSORS_H

#include "Shared.h"
#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
#include "DataStore.h"

class Sensors {
public:
    Sensors(DataStore& dataStore);
    void begin();
    void read();

private:
    DataStore& data;
    SHT4xSensor sht4xSensor;
    ThermistorSensor thermistorSensor;
};

#endif // SENSORS_H