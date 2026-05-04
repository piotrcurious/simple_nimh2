#ifndef INTERNAL_RESISTANCE_H
#define INTERNAL_RESISTANCE_H

#include "definitions.h"

void measureInternalResistanceStep();
void handleGeneratePairs();
void handlePairGeneration();
void handleMeasureLoadedUnloaded();
void handleMeasurePairs();
void completeResistanceMeasurement();
void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count);

#endif
