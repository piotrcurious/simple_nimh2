#ifndef GRAPHING_H
#define GRAPHING_H

#include "definitions.h"

enum DisplayState {
    DISPLAY_STATE_MAIN,
    DISPLAY_STATE_IR_GRAPH,
    DISPLAY_STATE_CHARGE_GRAPH
};

extern DisplayState currentDisplayState;

// Function declarations for graphing
void prepareTemperaturePlot();
void plotVoltageData();
void plotTemperatureData();
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
float mapf(float value, float in_min, float in_max, float out_min, float out_max);
void bigUglyMessage(const String& measurementType);


#endif // GRAPHING_H
