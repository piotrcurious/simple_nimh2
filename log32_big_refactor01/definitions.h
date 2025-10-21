#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <TFT_eSPI.h>
#include <SPI.h>

#include <cmath>
#include <limits>
#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <ArduinoEigenDense.h>
#include <cstdlib>
#include <map>
#include <ctime>
#include <Arduino.h>

#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
#include "analog.h"

// All constants and type definitions have been moved to config.h and Shared.h

// --- Extern Global Variables ---
extern TFT_eSPI tft;
extern SHT4xSensor sht4Sensor;
extern ThermistorSensor thermistorSensor;
extern AsyncMeasure meas;
extern FindOptManager findOpt;
// All other state variables have been moved to the DataStore class.


// --- Function Declarations ---
// These are functions that are defined in one .cpp file but called from another.
// This helps avoid circular dependencies.

// from main .ino
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
void buildCurrentModel(bool warmStart);
float estimateCurrent(int dutyCycle);
MeasurementData takeMeasurement(int dc, uint32_t stabilization_delay);
void processThermistorData(const MeasurementData& data, const String& measurementType);
inline unsigned long unmanagedCastUL(unsigned long v){ return v; }


// from graphing.cpp
void drawChargePlot(bool autoscaleX, bool autoscaleY);
void plotTemperatureData();
void prepareTemperaturePlot();
void plotVoltageData();
void displayInternalResistanceGraph();
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);

// from internal_resistance.cpp
void measureInternalResistance();
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept);
void bubbleSort(float data[][2], int n);
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count);
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier);


// from charging.cpp
void startCharging();
void stopCharging();
void handleBatteryCharging();
bool chargeBattery();
void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay);
bool measurementStep();
bool fetchMeasurementResult(MHElectrodeData &out);
void abortMeasurement();
void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval);
bool findOptimalChargingDutyCycleStepAsync();
float estimateTempDiff(float voltageUnderLoad, float voltageNoLoad, float current, float internalResistanceParam, float ambientTempC, uint32_t currentTime, uint32_t lastChargeEvaluationTime, float BatteryTempC, float cellMassKg, float specificHeat, float area, float convectiveH, float emissivity);


#endif // DEFINITIONS_H
