#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <vector>

// Structure to hold charge log data
struct ChargeLogData {
    unsigned long timestamp;
    float current;
    float voltage;
    float ambientTemperature;
    float batteryTemperature;
    int dutyCycle;
    float internalResistanceLoadedUnloaded;
    float internalResistancePairs;
};

// Extern declaration for the global charge log vector
extern std::vector<ChargeLogData> chargeLog;

// Function declaration for logging
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);

#endif // LOGGING_H