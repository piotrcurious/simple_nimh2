#ifndef INTERNAL_RESISTANCE_H
#define INTERNAL_RESISTANCE_H

#include "definitions.h"

enum IRState {
    IR_STATE_IDLE,
    IR_STATE_START,
    IR_STATE_STOP_LOAD_WAIT,
    IR_STATE_GET_UNLOADED_VOLTAGE,
    IR_STATE_FIND_MIN_DC,
    IR_STATE_GENERATE_PAIRS,
    IR_STATE_MEASURE_L_UL,
    IR_STATE_MEASURE_PAIRS,
    IR_STATE_GET_MEASUREMENT,
    IR_STATE_EVALUATE,
    IR_STATE_REMEASURE_OUTLIERS,
    IR_STATE_FINAL_CALCULATION
};

extern IRState currentIRState;

void handleGeneratePairs();
void handleMeasurePairs();
void handlePairGeneration();
void handleMeasureLoadedUnloaded();
void measureInternalResistanceStep();
void buildCurrentToDutyCycleModel();
void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count);
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count);
void bubbleSort(float data[][2], int n);
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept);
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier);


#endif // INTERNAL_RESISTANCE_H
