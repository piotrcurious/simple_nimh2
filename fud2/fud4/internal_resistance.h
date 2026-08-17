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

// Duty Cycle Model non-linear region prediction, blind-spot discovery, and pair generation helpers
bool isDutyCycleLinearRegion(int dc, float& out_slope);
void generateCategorizedDutyPairs(std::vector<DutyPair>& pairs, int maxPairs);
void findCurrentBlindSpots(float gaps[][2], int& gapCount, float maxOperatingCurrent);
bool evaluateAndCorrectPairData(int dc1, int dc2, float v1, float v2, float i1, float i2, float& out_I, float& out_IR);
bool evaluateElectrodeParameters(const ElectrodeTransient &measurement);
bool evaluateElectrodeParameters(
    const float *time_s,
    const float *voltage_V,
    const float *current_A,
    size_t count,
    float v_unloaded,
    float i_before_A,
    float specificCdl_F_per_m2 = -1.0f);

// Coverage tracking, interval midpoint discovery, and budget candidate allocator
bool isCurrentCovered(float current, float tolerance);
void computeIntervalMidpointsAndBlindSpots(std::vector<CandidatePoint>& candidates, float minI, float maxI, float activeCurrent);

// Structure definition for PulseIRRemeasure points
struct RePoint {
    float current;
    int duty;
    bool isPair;
};

void allocateReevaluationCandidates(std::vector<RePoint>& points, int budget);

#endif
