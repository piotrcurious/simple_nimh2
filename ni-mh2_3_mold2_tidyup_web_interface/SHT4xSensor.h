// SHT4xSensor.h
#ifndef SHT4X_SENSOR_H
#define SHT4X_SENSOR_H

#include <Adafruit_SHT4x.h>
#include <stdint.h>

class SHT4xSensor {
public:
    SHT4xSensor();
    bool begin();
    void read();
    double getTemperature();
    double getHumidity();
    void setPrecision(sht4x_precision_t precision);
    sht4x_precision_t getPrecision();
    void setHeater(sht4x_heater_t heater);
    sht4x_heater_t getHeater();
    bool isLocked();

private:
    Adafruit_SHT4x sht4;
    uint32_t last_time;
    uint32_t update_interval;
    bool lock;
    double temperature;
    double humidity;
};

#endif // SHT4X_SENSOR_H
