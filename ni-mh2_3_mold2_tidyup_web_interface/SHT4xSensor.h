#ifndef SHT4X_SENSOR_H
#define SHT4X_SENSOR_H

#ifndef MOCK_TEST
#include <Adafruit_SHT4x.h>
#else
#include "test_mock/Adafruit_SHT4x.h"
#endif

class SHT4xSensor {
public:
    SHT4xSensor();
    bool begin();
    void read();
    void setPrecision(sht4x_precision_t precision);
    void setHeater(sht4x_heater_t heater);

    float getTemperature() const { return _temperature; }
    float getHumidity() const { return _humidity; }

private:
#ifndef MOCK_TEST
    Adafruit_SHT4x _sht4;
#endif
    float _temperature;
    float _humidity;
};

#endif // SHT4X_SENSOR_H
