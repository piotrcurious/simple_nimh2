#include "logging.h"
#include <vector>
#include <Arduino.h>

// Global vector to store charge log data
std::vector<ChargeLogData> chargeLog;

// Function to print thermistor, voltage, and current data to the serial monitor
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current) {
    Serial.print("Thermistor 1 (Top, SHT4x): ");
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.3f °C", temp1);
    Serial.print(", Thermistor 2 (Bottom): ");
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.3f °C", temp2);
    Serial.print(", Diff (T1-T2): ");
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.3f °C", tempDiff);
    Serial.printf(", Voltage : %.3f V", voltage);
    Serial.printf(", Current : %.3f A", current);
    Serial.println(" °C");
}