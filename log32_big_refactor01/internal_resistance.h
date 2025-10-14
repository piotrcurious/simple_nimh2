#ifndef INTERNAL_RESISTANCE_H
#define INTERNAL_RESISTANCE_H

#include "definitions.h"

void measureInternalResistance();
void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count);
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count);
void bubbleSort(float data[][2], int n);
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept);
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier);


#endif // INTERNAL_RESISTANCE_H
