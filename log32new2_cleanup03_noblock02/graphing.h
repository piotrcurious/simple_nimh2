#ifndef GRAPHING_H
#define GRAPHING_H

#include "definitions.h"

// Function declarations for graphing
void prepareTemperaturePlot();
void plotVoltageData();
void plotTemperatureData();
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
void displayInternalResistanceGraph();
void drawChargePlot(bool autoscaleX, bool autoscaleY);
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
float mapf(float value, float in_min, float in_max, float out_min, float out_max);
void bigUglyMessage(const String& measurementType);


#endif // GRAPHING_H