#include "internal_resistance.h"
#include "definitions.h"
#include "AdvancedPolynomialFitter.hpp"
#include <algorithm> // for nth_element
#include <cmath>

#ifndef MOCK_TEST
#define WEB_LOCK() if (webDataMutex) xSemaphoreTake(webDataMutex, portMAX_DELAY)
#define WEB_UNLOCK() if (webDataMutex) xSemaphoreGive(webDataMutex)
#else
#define WEB_LOCK()
#define WEB_UNLOCK()
#endif

// External functions
extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff,
                                   float& t1_millivolts, float& voltage, float& current);

// State machine variables
volatile IRState currentIRState = IR_STATE_IDLE;
IRState nextIRState = IR_STATE_IDLE;
unsigned long irStateChangeTime = 0;
MeasurementData currentMeasurement;

// Duty cycle search variables
int minimalDutyCycle = 0;
int findMinDcLow = 0;
int findMinDcHigh = 0;
int findMinDcMid = 0;

// Pair generation variables
std::vector<std::pair<int, int>> dutyCyclePairs;
int pairIndex = 0;
int pairGenerationStep = 0;
int pairGenerationSubStep = 0;
int lowDc = 0;
int previousHighDc = 0;
int lowBound = 0;
int highBound = 0;
int bestHighDc = 0;
float minCurrent = 0.0f;
float maxCurrent = 0.0f;
float minCurrentDifference = 0.0f;

// Measurement variables
int measureStep = 0;
std::vector<float> voltagesLoaded;
std::vector<float> currentsLoaded;
std::vector<float> ir_dutyCycles;
std::vector<float> consecutiveInternalResistances;

// Results storage
float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;

// Regression results
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

bool isMeasuringResistance = false;

// Helper function to initiate measurement
void getSingleMeasurement(int dc, IRState nextState) {
    applyDuty(dc);
    irStateChangeTime = millis();
    nextIRState = nextState;
    currentIRState = IR_STATE_GET_MEASUREMENT;
}

// Reset all state variables for new measurement
void resetMeasurementState() {
    WEB_LOCK();
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    WEB_UNLOCK();

    voltagesLoaded.clear();
    voltagesLoaded.reserve(MAX_RESISTANCE_POINTS);

    currentsLoaded.clear();
    currentsLoaded.reserve(MAX_RESISTANCE_POINTS);

    ir_dutyCycles.clear();
    ir_dutyCycles.reserve(MAX_RESISTANCE_POINTS);

    consecutiveInternalResistances.clear();
    consecutiveInternalResistances.reserve(MAX_RESISTANCE_POINTS);

    dutyCyclePairs.clear();
    dutyCyclePairs.reserve(MAX_RESISTANCE_POINTS / 2);

    pairIndex = 0;
    measureStep = 0;
    pairGenerationStep = 0;
    pairGenerationSubStep = 0;

    findMinDcLow = 0;
    findMinDcHigh = 0;
    findMinDcMid = 0;
    minimalDutyCycle = 0;

    minCurrent = 0.0f;
    maxCurrent = 0.0f;
    lowDc = 0;
    previousHighDc = 0;
    bestHighDc = 0;
    minCurrentDifference = 0.0f;
}

void measureInternalResistanceStep() {
    unsigned long now = millis();

    switch (currentIRState) {
        case IR_STATE_IDLE:
            break;

        case IR_STATE_START:
            Serial.println("Starting improved internal resistance measurement...");
            resetMeasurementState();
            currentIRState = IR_STATE_STOP_LOAD_WAIT;
            irStateChangeTime = now;
            applyDuty(0);
            break;

        case IR_STATE_STOP_LOAD_WAIT:
            if (now - irStateChangeTime >= UNLOADED_VOLTAGE_DELAY_MS) {
                currentIRState = IR_STATE_GET_UNLOADED_VOLTAGE;
            }
            break;

        case IR_STATE_GET_UNLOADED_VOLTAGE:
            {
                MeasurementData initialUnloaded;
                getThermistorReadings(initialUnloaded.temp1, initialUnloaded.temp2,
                                     initialUnloaded.tempDiff, initialUnloaded.t1_millivolts,
                                     initialUnloaded.voltage, initialUnloaded.current);
                Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;

                Serial.println("Finding minimal duty cycle...");
                currentIRState = IR_STATE_FIND_MIN_DC;
                findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                getSingleMeasurement(findMinDcMid, IR_STATE_FIND_MIN_DC);
            }
            break;

        case IR_STATE_FIND_MIN_DC:
            if (currentMeasurement.dutyCycle == findMinDcMid) {
                if (currentMeasurement.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    minimalDutyCycle = findMinDcMid;
                    findMinDcHigh = findMinDcMid - 1;
                } else {
                    findMinDcLow = findMinDcMid + 1;
                }

                if (findMinDcLow <= findMinDcHigh) {
                    findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                    getSingleMeasurement(findMinDcMid, IR_STATE_FIND_MIN_DC);
                } else {
                    if (minimalDutyCycle > 0) {
                        Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
                        currentIRState = IR_STATE_GENERATE_PAIRS;
                    } else {
                        Serial.println("Warning: No measurable current found.");
                        currentIRState = IR_STATE_IDLE;
                        isMeasuringResistance = false;
                    }
                }
            }
            break;

        case IR_STATE_GENERATE_PAIRS:
            handleGeneratePairs();
            break;

        case IR_STATE_MEASURE_L_UL:
            handleMeasureLoadedUnloaded();
            break;

        case IR_STATE_MEASURE_PAIRS:
            handleMeasurePairs();
            break;

        case IR_STATE_GET_MEASUREMENT:
            if (now - irStateChangeTime >= STABILIZATION_DELAY_MS) {
                getThermistorReadings(currentMeasurement.temp1, currentMeasurement.temp2,
                                     currentMeasurement.tempDiff, currentMeasurement.t1_millivolts,
                                     currentMeasurement.voltage, currentMeasurement.current);
                currentMeasurement.dutyCycle = (uint8_t)dutyCycle;
                currentMeasurement.timestamp = (uint32_t)millis();
                currentIRState = nextIRState;
            }
            break;

        case IR_STATE_COMPLETE:
            completeResistanceMeasurement();
            break;
    }
}

void handleGeneratePairs() {
    switch (pairGenerationStep) {
        case 0:
            if (minimalDutyCycle == 0) {
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            getSingleMeasurement(minimalDutyCycle, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 1;
            break;

        case 1:
            minCurrent = currentMeasurement.current;
            getSingleMeasurement(MAX_DUTY_CYCLE, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 2;
            break;

        case 2:
            maxCurrent = currentMeasurement.current;
            if (maxCurrent <= minCurrent) {
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            lowDc = minimalDutyCycle;
            previousHighDc = MAX_DUTY_CYCLE;
            pairIndex = 0;
            pairGenerationStep = 3;
            pairGenerationSubStep = 0;
            // Fall through

        case 3:
            handlePairGeneration();
            break;
    }
}

void handlePairGeneration() {
    if (pairGenerationSubStep == 0) {
        if (pairIndex < MAX_RESISTANCE_POINTS / 2) {
            float targetHighCurrent = maxCurrent -
                (pairIndex * (maxCurrent - minCurrent) / (MAX_RESISTANCE_POINTS / 2));
            bestHighDc = -1;
            minCurrentDifference = 1e9f;
            lowBound = minimalDutyCycle;
            highBound = previousHighDc;
            pairGenerationSubStep = 1;
        } else {
            currentIRState = IR_STATE_MEASURE_L_UL;
            pairIndex = 0;
            measureStep = 0;
        }
    } else if (pairGenerationSubStep == 1) {
        if (lowBound <= highBound) {
            int midDc = lowBound + (highBound - lowBound) / 2;
            getSingleMeasurement(midDc, IR_STATE_GENERATE_PAIRS);
            pairGenerationSubStep = 2;
        } else {
            int currentHighDc = (bestHighDc != -1) ? bestHighDc : previousHighDc;
            currentHighDc = std::max(currentHighDc, minimalDutyCycle);
            dutyCyclePairs.push_back({lowDc, currentHighDc});
            previousHighDc = currentHighDc - 1;
            pairIndex++;
            pairGenerationSubStep = 0;
        }
    } else if (pairGenerationSubStep == 2) {
        float targetHighCurrent = maxCurrent -
            (pairIndex * (maxCurrent - minCurrent) / (MAX_RESISTANCE_POINTS / 2));
        float currentDifference = std::fabs(currentMeasurement.current - targetHighCurrent);
        if (currentDifference < minCurrentDifference) {
            minCurrentDifference = currentDifference;
            bestHighDc = currentMeasurement.dutyCycle;
        }
        if (currentMeasurement.current > targetHighCurrent) {
            highBound = currentMeasurement.dutyCycle - 1;
        } else {
            lowBound = currentMeasurement.dutyCycle + 1;
        }
        pairGenerationSubStep = 1;
    }
}

void handleMeasureLoadedUnloaded() {
    if (pairIndex >= (int)dutyCyclePairs.size()) {
        currentIRState = IR_STATE_MEASURE_PAIRS;
        pairIndex = 0;
        measureStep = 0;
        return;
    }
    switch (measureStep) {
        case 0:
            getSingleMeasurement(dutyCyclePairs[pairIndex].second, IR_STATE_MEASURE_L_UL);
            measureStep = 1;
            break;
        case 1:
            voltagesLoaded.push_back(currentMeasurement.voltage);
            currentsLoaded.push_back(currentMeasurement.current);
            ir_dutyCycles.push_back(static_cast<float>(currentMeasurement.dutyCycle));
            getSingleMeasurement(0, IR_STATE_MEASURE_L_UL);
            measureStep = 2;
            break;
        case 2:
            {
                float loadedCurrent = currentsLoaded.back();
                if (loadedCurrent > 0.01f) {
                    float internalResistance = (currentMeasurement.voltage - voltagesLoaded.back()) / loadedCurrent;
                    WEB_LOCK();
                    storeResistanceData(loadedCurrent, std::fabs(internalResistance),
                                      internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                    WEB_UNLOCK();
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void handleMeasurePairs() {
    if (pairIndex >= (int)dutyCyclePairs.size()) {
        currentIRState = IR_STATE_COMPLETE;
        return;
    }
    switch (measureStep) {
        case 0:
            getSingleMeasurement(dutyCyclePairs[pairIndex].first, IR_STATE_MEASURE_PAIRS);
            measureStep = 1;
            break;
        case 1:
            voltagesLoaded.push_back(currentMeasurement.voltage);
            currentsLoaded.push_back(currentMeasurement.current);
            getSingleMeasurement(dutyCyclePairs[pairIndex].second, IR_STATE_MEASURE_PAIRS);
            measureStep = 2;
            break;
        case 2:
            {
                float currentDiff = currentMeasurement.current - currentsLoaded.back();
                if (currentDiff > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                    float voltageDiff = voltagesLoaded.back() - currentMeasurement.voltage;
                    float internalResistance = voltageDiff / currentDiff;
                    consecutiveInternalResistances.push_back(std::fabs(internalResistance));
                    WEB_LOCK();
                    storeResistanceData(currentMeasurement.current, std::fabs(internalResistance),
                                      internalResistanceDataPairs, resistanceDataCountPairs);
                    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    WEB_UNLOCK();
                } else {
                    consecutiveInternalResistances.push_back(-1.0f);
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void completeResistanceMeasurement() {
    WEB_LOCK();
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    auto compute_spacing_threshold = [](float data[][2], int n) -> float {
        if (n < 2) return 0.05f;
        float minX = data[0][0], maxX = data[n-1][0];
        float avgSpacing = (maxX - minX) / std::max(1, n-1);
        return std::max(0.02f, avgSpacing * 1.5f);
    };

    distribute_error(internalResistanceData, resistanceDataCount, compute_spacing_threshold(internalResistanceData, resistanceDataCount), 1.5f);
    distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, compute_spacing_threshold(internalResistanceDataPairs, resistanceDataCountPairs), 1.5f);

    if (resistanceDataCount >= 2) {
        performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
    }
    if (resistanceDataCountPairs >= 2) {
        performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
    }
    WEB_UNLOCK();
    isMeasuringResistance = false;
    currentIRState = IR_STATE_IDLE;
}

void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        bool swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                float temp0 = data[j][0], temp1 = data[j][1];
                data[j][0] = data[j + 1][0]; data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0; data[j + 1][1] = temp1;
                swapped = true;
            }
        }
        if (!swapped) break;
    }
}

void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count >= MAX_RESISTANCE_POINTS) return;
    if (resistance > MIN_VALID_RESISTANCE && resistance < 1000.0f) {
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    int low = 0, high = count - 1;
    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (std::fabs(data[mid][0] - targetCurrent) < 1e-6f) return mid;
        else if (data[mid][0] < targetCurrent) low = mid + 1;
        else high = mid - 1;
    }
    if (low >= count) return high;
    if (high < 0) return low;
    return (std::fabs(targetCurrent - data[high][0]) < std::fabs(data[low][0] - targetCurrent)) ? high : low;
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count >= MAX_RESISTANCE_POINTS || index < 0 || index > count) return;
    for (int i = count; i > index; --i) {
        data[i][0] = data[i - 1][0]; data[i][1] = data[i - 1][1];
    }
    data[index][0] = current; data[index][1] = resistance;
    count++;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index < 0 || index >= count) return;
    for (int i = index; i < count - 1; ++i) {
        data[i][0] = data[i + 1][0]; data[i][1] = data[i + 1][1];
    }
    count--;
}

void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= MIN_VALID_RESISTANCE || resistance >= 1000.0f || current < 0.0f) return;
    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
        insertDataPoint(data, count, current, resistance, insertIndex);
        return;
    }
    int closestIndex = findClosestIndex(data, count, current);
    const float CLOSE_TOLERANCE = 1e-3f * std::max(1.0f, current);
    if (std::fabs(data[closestIndex][0] - current) <= CLOSE_TOLERANCE) {
        const float alpha = 0.5f;
        data[closestIndex][1] = alpha * resistance + (1.0f - alpha) * data[closestIndex][1];
        data[closestIndex][0] = alpha * current + (1.0f - alpha) * data[closestIndex][0];
        return;
    }
    int evictIndex = 0;
    float maxGap = -1.0f;
    for (int i = 0; i < count; ++i) {
        float leftGap = (i > 0) ? data[i][0] - data[i-1][0] : 0.0f;
        float rightGap = (i < count-1) ? data[i+1][0] - data[i][0] : 0.0f;
        float localGap = std::max(leftGap, rightGap);
        if (localGap > maxGap) { maxGap = localGap; evictIndex = i; }
    }
    removeDataPoint(data, count, evictIndex);
    int insertIndex = 0;
    while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
    insertDataPoint(data, count, current, resistance, insertIndex);
}

float computeMedian(std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    size_t n = v.size();
    size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float med = v[mid];
    if (n % 2 == 0) {
        auto it = std::max_element(v.begin(), v.begin() + mid);
        med = (med + *it) / 2.0f;
    }
    return med;
}

void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return;
    static std::vector<float> res;
    res.reserve(MAX_RESISTANCE_POINTS);
    for (int i = 0; i <= count - 4; ++i) {
        for (int j = i + 3; j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                res.clear();
                for (int k = i; k <= j; ++k) res.push_back(data[k][1]);
                if (res.size() >= 4) {
                    float median = computeMedian(res), sumSq = 0.0f;
                    for (float r : res) { float d = r - median; sumSq += d * d; }
                    float stdDev = std::sqrt(sumSq / res.size()), errorThreshold = error_threshold_multiplier * stdDev;
                    const float alpha = 0.6f;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - median) > errorThreshold && stdDev > 1e-9f) {
                            data[k][1] = alpha * median + (1.0f - alpha) * data[k][1];
                        }
                    }
                    i = j; break;
                }
            } else break;
        }
    }
}

bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) return false;
#ifndef MOCK_TEST
    std::vector<float> x(count), y(count);
    for (int i = 0; i < count; ++i) {
        x[i] = data[i][0];
        y[i] = data[i][1];
    }
    AdvancedPolynomialFitter fitter;
    std::vector<float> coeffs = fitter.fitPolynomialLebesgue(x, y, 1);
    if (coeffs.size() >= 2) {
        intercept = coeffs[0];
        slope = coeffs[1];
        return true;
    }
    return false;
#else
    slope = 0; intercept = 0.2f; return true;
#endif
}
