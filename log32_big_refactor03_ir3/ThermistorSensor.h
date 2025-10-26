// ThermistorSensor.h
#ifndef THERMISTOR_SENSOR_H
#define THERMISTOR_SENSOR_H

#include <Arduino.h>
#include <cmath>
#include <limits>
#include "analog.h"

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
//    void setVccMillivolts(double vcc);
    bool isLocked();

private:
    // Thermistor parameters - ADJUST THESE BASED ON YOUR THERMISTOR DATASHEET
    const double THERMISTORNOMINAL = 10000.0; // Resistance at 25 degrees C (Ohms)
    const double TEMPERATURENOMINAL = 25.0;    // Temperature for nominal resistance (degrees C)
    const double BCOEFFICIENT = 3950.0;       // Beta coefficient (from datasheet)
    const double SERIESRESISTOR = 10000.0;     // Series resistor value in voltage divider circuit (Ohms)
#define MAIN_VCC_RATIO 2.0 // multiplier to determine VCC from thermistor divider VCC

    double vcc_millivolts;
    double thermistor1Value;
    double thermistor2Value;
    double thermistorDiffValue;
    double thermistorVccValue;
    float thermistor1RawMillivolts;
    bool lock;
    uint32_t last_time;
    const uint32_t update_interval = 500; // Adjust as needed
    int thermistor1Pin;
    int thermistorVccPin;
    double thermistor1Offset;

    double readVCCInternal(int pin, int numSamples = 128);
    double readThermistorInternal(int pin, int numSamples = 1);
    double readThermistorRelativeInternal(int pin, double topThermistorTemperatureCelsius, int numSamples = 1, double VCC_offset = 0);
    double resistanceToTemperature(double resistance);
    double temperatureToResistance(double temperatureCelsius);
    float mapfInternal(float value, float in_min, float in_max, float out_min, float out_max);
};

#endif // THERMISTOR_SENSOR_H
