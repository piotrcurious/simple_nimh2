// ThermistorSensor.h
#ifndef THERMISTOR_SENSOR_H
#define THERMISTOR_SENSOR_H

#include <Arduino.h>
#include <cmath>
#include <limits>
#include "analog.h"
#include "adc_dma.h"

class ThermistorSensor {
public:
    ThermistorSensor(int thermistor1Pin, int thermistorVccPin, double thermistor1Offset);
    void begin();
    void read(double topThermistorTemperatureCelsius);
    double getTemperature1();
    double getTemperature2();
    double getDifference();
    double getVCC();
    float getRawMillivolts1();
    bool isLocked();

private:
    // Thermistor parameters
    const double THERMISTORNOMINAL = 10000.0;
    const double TEMPERATURENOMINAL = 25.0;
    const double BCOEFFICIENT = 3950.0;
    const double SERIESRESISTOR = 10000.0;
#ifndef MAIN_VCC_RATIO
#define MAIN_VCC_RATIO 2.0
#endif

    double vcc_millivolts;
    double thermistor1Value;
    double thermistor2Value;
    double thermistorDiffValue;
    double thermistorVccValue;
    float thermistor1RawMillivolts;
    bool lock;
    uint32_t last_time;
    const uint32_t update_interval = 500;
    int thermistor1Pin;
    int thermistorVccPin;
    double thermistor1Offset;

    // Snapshot tracking for DMA superior oversampling
    AdcSnapshot last_snapshot_vcc;
    AdcSnapshot last_snapshot_therm1;

    double readVCCInternal(int pin);
    double readThermistorRelativeInternal(int pin, double topThermistorTemperatureCelsius, double offset);
    double resistanceToTemperature(double resistance);
    double temperatureToResistance(double temperatureCelsius);
    float mapfInternal(float value, float in_min, float in_max, float out_min, float out_max);
};

#endif // THERMISTOR_SENSOR_H
