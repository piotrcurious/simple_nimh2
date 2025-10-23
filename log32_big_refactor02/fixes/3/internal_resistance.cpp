#include "internal_resistance.h"
#include "definitions.h"

// Configuration constants
namespace IRConfig {
    constexpr float MAX_VALID_RESISTANCE = 1000.0f;
    constexpr float DEFAULT_ISOLATION_THRESHOLD = 0.02f;
    constexpr float ERROR_THRESHOLD_MULTIPLIER = 1.5f;
    constexpr float ZERO_THRESHOLD = 1e-6f;
    constexpr float REGRESSION_MIN_DENOMINATOR = 1e-6f;
    constexpr int MIN_REGRESSION_POINTS = 2;
    constexpr int MIN_CLUSTER_SIZE = 4;
}

// External functions
extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, 
                                   float& t1_millivolts, float& voltage, float& current);
extern void processThermistorData(const MeasurementData& data, const String& measurementType);

// State machine variables
IRState currentIRState = IR_STATE_IDLE;
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

// ============================================================================
// Helper Functions
// ============================================================================

// Safely set PWM and prepare for measurement
void getSingleMeasurement(int dc, IRState nextState) {
    // Clamp duty cycle to valid range
    dc = constrain(dc, 0, MAX_DUTY_CYCLE);
    dutyCycle = dc;
    analogWrite(pwmPin, dutyCycle);
    irStateChangeTime = millis();
    nextIRState = nextState;
    currentIRState = IR_STATE_GET_MEASUREMENT;
}

// Validate resistance value
inline bool isValidResistance(float resistance) {
    return (resistance > MIN_VALID_RESISTANCE && 
            resistance < IRConfig::MAX_VALID_RESISTANCE);
}

// Validate current value
inline bool isValidCurrent(float current) {
    return (current >= 0.0f && current < 1000.0f); // Reasonable upper limit
}

// Reset all state variables for new measurement
void resetMeasurementState() {
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    
    // Clear vectors and free memory
    voltagesLoaded.clear();
    voltagesLoaded.shrink_to_fit();
    currentsLoaded.clear();
    currentsLoaded.shrink_to_fit();
    ir_dutyCycles.clear();
    ir_dutyCycles.shrink_to_fit();
    consecutiveInternalResistances.clear();
    consecutiveInternalResistances.shrink_to_fit();
    dutyCyclePairs.clear();
    dutyCyclePairs.shrink_to_fit();
    
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

// ============================================================================
// Main State Machine
// ============================================================================

void measureInternalResistanceStep() {
    unsigned long now = millis();

    switch (currentIRState) {
        case IR_STATE_IDLE:
            // Waiting for start command
            break;

        case IR_STATE_START:
            Serial.println("Starting improved internal resistance measurement...");
            resetMeasurementState();
            currentIRState = IR_STATE_STOP_LOAD_WAIT;
            irStateChangeTime = now;
            analogWrite(pwmPin, 0);
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
                
                if (initialUnloaded.voltage <= 0.0f) {
                    Serial.println("Error: Invalid unloaded voltage");
                    currentIRState = IR_STATE_IDLE;
                    isMeasuringResistance = false;
                    break;
                }
                
                Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);
                
                // Initialize binary search parameters
                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;
                
                Serial.println("Finding minimal duty cycle for measurable current...");
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);
                tft.drawString("Finding Min DC", 10, 10);
                
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
                
                // Continue binary search or finish
                if (findMinDcLow <= findMinDcHigh) {
                    findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                    getSingleMeasurement(findMinDcMid, IR_STATE_FIND_MIN_DC);
                } else {
                    // Search complete
                    tft.fillScreen(TFT_BLACK);
                    tft.setTextColor(TFT_WHITE);
                    tft.setTextSize(2);
                    
                    if (minimalDutyCycle > 0) {
                        Serial.printf("Minimal duty cycle: %d\n", minimalDutyCycle);
                        tft.printf("Min DC: %d%%", minimalDutyCycle);
                        currentIRState = IR_STATE_GENERATE_PAIRS;
                    } else {
                        Serial.println("Error: No measurable current found");
                        tft.println("Error: No current");
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
                currentMeasurement.dutyCycle = dutyCycle;
                currentMeasurement.timestamp = millis();
                currentIRState = nextIRState;
            }
            break;

        case IR_STATE_COMPLETE:
            completeResistanceMeasurement();
            break;
    }
}

// ============================================================================
// State Handlers
// ============================================================================

void handleGeneratePairs() {
    switch (pairGenerationStep) {
        case 0:
            Serial.println("Generating duty cycle pairs...");
            if (minimalDutyCycle <= 0 || minimalDutyCycle > MAX_DUTY_CYCLE) {
                Serial.println("Error: Invalid minimal duty cycle");
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            getSingleMeasurement(minimalDutyCycle, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 1;
            break;
            
        case 1:
            minCurrent = currentMeasurement.current;
            if (!isValidCurrent(minCurrent)) {
                Serial.println("Error: Invalid min current reading");
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            Serial.printf("Min current at DC %d: %.3f A\n", minimalDutyCycle, minCurrent);
            getSingleMeasurement(MAX_DUTY_CYCLE, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 2;
            break;
            
        case 2:
            maxCurrent = currentMeasurement.current;
            if (!isValidCurrent(maxCurrent) || maxCurrent <= minCurrent) {
                Serial.println("Error: Invalid current range");
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            Serial.printf("Max current at DC %d: %.3f A\n", MAX_DUTY_CYCLE, maxCurrent);
            
            lowDc = minimalDutyCycle;
            previousHighDc = MAX_DUTY_CYCLE;
            pairIndex = 0;
            pairGenerationStep = 3;
            pairGenerationSubStep = 0;
            
            // Reserve memory for pairs
            dutyCyclePairs.reserve(MAX_RESISTANCE_POINTS / 2);
            // Fall through to next step
            
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
            // All pairs generated
            Serial.printf("Generated %d duty cycle pairs\n", dutyCyclePairs.size());
            
            // Reserve memory for measurement vectors
            voltagesLoaded.reserve(dutyCyclePairs.size() * 2);
            currentsLoaded.reserve(dutyCyclePairs.size() * 2);
            ir_dutyCycles.reserve(dutyCyclePairs.size() * 2);
            consecutiveInternalResistances.reserve(dutyCyclePairs.size());
            
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
            // Binary search complete for this pair
            int currentHighDc = (bestHighDc != -1) ? bestHighDc : previousHighDc;
            currentHighDc = max(currentHighDc, minimalDutyCycle);
            
            dutyCyclePairs.push_back({lowDc, currentHighDc});
            Serial.printf("Pair %d: Low=%d, High=%d\n", pairIndex, lowDc, currentHighDc);
            
            previousHighDc = currentHighDc - 1;
            pairIndex++;
            pairGenerationSubStep = 0;
        }
    } else if (pairGenerationSubStep == 2) {
        float targetHighCurrent = maxCurrent - 
            (pairIndex * (maxCurrent - minCurrent) / (MAX_RESISTANCE_POINTS / 2));
        float currentDifference = fabs(currentMeasurement.current - targetHighCurrent);

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
    if (pairIndex >= static_cast<int>(dutyCyclePairs.size())) {
        currentIRState = IR_STATE_MEASURE_PAIRS;
        pairIndex = 0;
        measureStep = 0;
        Serial.println("Loaded/unloaded measurements complete");
        return;
    }
    
    switch (measureStep) {
        case 0:
            {
                int dc = dutyCyclePairs[pairIndex].second;
                getSingleMeasurement(dc, IR_STATE_MEASURE_L_UL);
                measureStep = 1;
            }
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
                float voltageDrop = currentMeasurement.voltage - voltagesLoaded.back();
                
                if (loadedCurrent > 0.01f && fabs(voltageDrop) > IRConfig::ZERO_THRESHOLD) {
                    float internalResistance = voltageDrop / loadedCurrent;
                    
                    if (isValidResistance(fabs(internalResistance))) {
                        storeResistanceData(loadedCurrent, fabs(internalResistance), 
                                          internalResistanceData, resistanceDataCount);
                        Serial.printf("L/UL Point %d: I=%.3fA, R=%.4fΩ\n", 
                                    pairIndex, loadedCurrent, fabs(internalResistance));
                    } else {
                        Serial.printf("L/UL Point %d: Invalid resistance %.4fΩ\n", 
                                    pairIndex, fabs(internalResistance));
                    }
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void handleMeasurePairs() {
    if (pairIndex >= static_cast<int>(dutyCyclePairs.size())) {
        currentIRState = IR_STATE_COMPLETE;
        Serial.println("Pair measurements complete");
        return;
    }
    
    switch (measureStep) {
        case 0:
            {
                int dcLow = dutyCyclePairs[pairIndex].first;
                getSingleMeasurement(dcLow, IR_STATE_MEASURE_PAIRS);
                measureStep = 1;
            }
            break;
            
        case 1:
            voltagesLoaded.push_back(currentMeasurement.voltage);
            currentsLoaded.push_back(currentMeasurement.current);
            {
                int dcHigh = dutyCyclePairs[pairIndex].second;
                getSingleMeasurement(dcHigh, IR_STATE_MEASURE_PAIRS);
            }
            measureStep = 2;
            break;
            
        case 2:
            {
                float currentDiff = currentMeasurement.current - currentsLoaded.back();
                
                if (currentDiff > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                    float voltageDiff = voltagesLoaded.back() - currentMeasurement.voltage;
                    float internalResistance = voltageDiff / currentDiff;
                    
                    if (isValidResistance(fabs(internalResistance))) {
                        consecutiveInternalResistances.push_back(fabs(internalResistance));
                        storeResistanceData(currentMeasurement.current, fabs(internalResistance), 
                                          internalResistanceDataPairs, resistanceDataCountPairs);
                        Serial.printf("Pair %d: ΔI=%.3fA, R=%.4fΩ\n", 
                                    pairIndex, currentDiff, fabs(internalResistance));
                    } else {
                        consecutiveInternalResistances.push_back(-1.0f);
                        Serial.printf("Pair %d: Invalid resistance %.4fΩ\n", 
                                    pairIndex, fabs(internalResistance));
                    }
                } else {
                    consecutiveInternalResistances.push_back(-1.0f);
                    Serial.printf("Pair %d: Insufficient ΔI (%.3fA)\n", pairIndex, currentDiff);
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void completeResistanceMeasurement() {
    Serial.println("\n=== Measurement Complete ===");
    
    // Sort data by current (ascending)
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    // Perform regression on loaded/unloaded data
    if (resistanceDataCount >= IRConfig::MIN_REGRESSION_POINTS) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount, 
                                    regressedInternalResistanceSlope, 
                                    regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed IR (L/UL): Slope=%.4f Ω/A, Intercept=%.4f Ω\n",
                         regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
    } else {
        Serial.printf("Insufficient L/UL data for regression (%d points)\n", resistanceDataCount);
    }

    // Perform regression on paired data
    if (resistanceDataCountPairs >= IRConfig::MIN_REGRESSION_POINTS) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                    regressedInternalResistancePairsSlope, 
                                    regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed IR (Pairs): Slope=%.4f Ω/A, Intercept=%.4f Ω\n",
                         regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.printf("Insufficient pairs data for regression (%d points)\n", resistanceDataCountPairs);
    }

    Serial.printf("\nCollected %d L/UL points, %d pair points\n", 
                 resistanceDataCount, resistanceDataCountPairs);
    
    // Calculate average if we have consecutive measurements
    if (!consecutiveInternalResistances.empty()) {
        calculateAverageInternalResistance(consecutiveInternalResistances);
    }
    
    isMeasuringResistance = false;
    currentIRState = IR_STATE_IDLE;
}

// ============================================================================
// Data Processing Functions
// ============================================================================

void bubbleSort(float data[][2], int n) {
    if (n <= 1) return;
    
    for (int i = 0; i < n - 1; i++) {
        bool swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                // Swap current values
                float temp0 = data[j][0];
                float temp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0;
                data[j + 1][1] = temp1;
                swapped = true;
            }
        }
        if (!swapped) break;
    }
}

void storeResistanceData(float current, float resistance, 
                        float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count >= MAX_RESISTANCE_POINTS) {
        Serial.println("Warning: Resistance data array full");
        return;
    }
    
    if (!isValidResistance(resistance)) {
        return;
    }
    
    if (!isValidCurrent(current)) {
        Serial.println("Warning: Invalid current value");
        return;
    }

    dataArray[count][0] = current;
    dataArray[count][1] = resistance;
    count++;
}

float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    if (resistances.empty()) {
        Serial.println("\nNo resistance measurements available");
        return -1.0f;
    }
    
    float sum = 0.0f;
    int count = 0;
    
    for (float rint : resistances) {
        if (isValidResistance(rint)) {
            sum += rint;
            count++;
        }
    }
    
    if (count > 0) {
        float average = sum / count;
        Serial.printf("\nAverage Internal Resistance: %.4f Ω (%d/%zu valid points)\n", 
                     average, count, resistances.size());
        return average;
    } else {
        Serial.println("\nNo valid internal resistance measurements");
        return -1.0f;
    }
}

bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < IRConfig::MIN_REGRESSION_POINTS) {
        Serial.printf("Error: Need at least %d points for regression (have %d)\n", 
                     IRConfig::MIN_REGRESSION_POINTS, count);
        return false;
    }

    // Calculate sums
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumXY = 0.0f;
    float sumX2 = 0.0f;

    for (int i = 0; i < count; ++i) {
        float x = data[i][0]; // Current
        float y = data[i][1]; // Resistance
        
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float n = static_cast<float>(count);
    float denominator = n * sumX2 - sumX * sumX;

    if (fabs(denominator) < IRConfig::REGRESSION_MIN_DENOMINATOR) {
        Serial.println("Error: Singular matrix in regression");
        return false;
    }

    // Calculate slope and intercept
    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / n;

    // Calculate R-squared
    float meanY = sumY / n;
    float ssTotal = 0.0f;
    float ssResidual = 0.0f;
    
    for (int i = 0; i < count; ++i) {
        float x = data[i][0];
        float y = data[i][1];
        float yPred = slope * x + intercept;
        
        float diffMean = y - meanY;
        float diffPred = y - yPred;
        ssTotal += diffMean * diffMean;
        ssResidual += diffPred * diffPred;
    }
    
    float rSquared = (ssTotal > IRConfig::ZERO_THRESHOLD) ? 
                     (1.0f - ssResidual / ssTotal) : 0.0f;
    Serial.printf("  R² = %.4f\n", rSquared);
    
    return true;
}

float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;
    
    // Calculate mean
    float sum = 0.0f;
    for (float val : data) {
        sum += val;
    }
    float mean = sum / data.size();
    
    // Calculate variance
    float sqSum = 0.0f;
    for (float val : data) {
        float diff = val - mean;
        sqSum += diff * diff;
    }
    
    return sqrt(sqSum / data.size());
}

// ============================================================================
// Display Functions
// ============================================================================

void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
    const uint16_t GREY = TFT_DARKGREY;
    const int BAR_Y = 10;
    const int BAR_HEIGHT = 10;
    const int BAR_START_X = 30;
    const int BAR_END_X = SCREEN_WIDTH - 2;
    const int BAR_WIDTH = BAR_END_X - BAR_START_X + 1;

    // Validate inputs
    if (BAR_WIDTH <= 0) return;
    
    // Draw background
    tft.fillRect(BAR_START_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT, GREY);
    tft.drawRect(BAR_START_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT, TFT_WHITE);

    int range = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
    if (range <= 0) range = 1;

    // Map and clamp coordinates
    int low_x = constrain(
        map(low, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X),
        BAR_START_X, BAR_END_X);
    int high_x = constrain(
        map(high, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X),
        BAR_START_X, BAR_END_X);
    int mid_x = constrain(
        map(mid, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X),
        BAR_START_X, BAR_END_X);

    // Draw range markers
    tft.drawLine(low_x, BAR_Y - 5, low_x, BAR_Y + BAR_HEIGHT + 5, TFT_WHITE);
    tft.drawLine(high_x, BAR_Y - 5, high_x, BAR_Y + BAR_HEIGHT + 5, TFT_WHITE);

    // Draw current position
    uint16_t mid_color = (current >= threshold) ? TFT_GREEN : TFT_RED;
    tft.fillCircle(mid_x, BAR_Y + BAR_HEIGHT / 2, 6, mid_color);

    // Display info
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, BAR_Y + BAR_HEIGHT + 15);
    tft.printf("L:%d%% H:%d%% M:%d%%", low, high, mid);
    tft.setCursor(10, BAR_Y + BAR_HEIGHT + 30);
    tft.printf("I:%.3fA Thr:%.3fA %s", 
              current, threshold, (current >= threshold) ? "OK" : "LOW");
}

void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, 
               int x, int y, int width, int height) {
    // Validate inputs
    if (numPoints <= 1 || width <= 0 || height <= 0 || 
        dutyCycles == nullptr || currents == nullptr) {
        return;
    }

    // Draw frame
    tft.fillRect(x, y, width, height, TFT_BLACK);
    tft.drawRect(x, y, width, height, TFT_WHITE);

    // Calculate scales
    int dcRange = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
    if (dcRange <= 0) dcRange = 1;
    
    float xScale = static_cast<float>(width) / dcRange;
    float yScale = (maxCurrent > IRConfig::ZERO_THRESHOLD) ? 
                   static_cast<float>(height) / maxCurrent : 0.0f;

    if (yScale <= 0.0f) return;

    // Draw data points and lines
    for (int i = 0; i < numPoints - 1; i++) {
        int x1 = x + static_cast<int>((dutyCycles[i] - MIN_DUTY_CYCLE_START) * xScale);
        int y1 = y + height - static_cast<int>(currents[i] * yScale);
        int x2 = x + static_cast<int>((dutyCycles[i + 1] - MIN_DUTY_CYCLE_START) * xScale);
        int y2 = y + height - static_cast<int>(currents[i + 1] * yScale);
        
        // Clamp to graph bounds
        x1 = constrain(x1, x, x + width);
        x2 = constrain(x2, x, x + width);
        y1 = constrain(y1, y, y + height);
        y2 = constrain(y2, y, y + height);
        
        tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
    }

    // Draw axes labels
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextSize(1);
    
    // X-axis label
    tft.setCursor(x + width / 2 - 20, y + height + 5);
    tft.print("DC (%)");
    
    // Y-axis label
    tft.setCursor(x - 25, y + height / 2);
    tft.print("I(A)");

    // Draw x-axis ticks
    int tickInterval = max(1, dcRange / 5);
    for (int i = MIN_DUTY_CYCLE_START; i <= MAX_DUTY_CYCLE; i += tickInterval) {
        int xTick = x + static_cast<int>((i - MIN_DUTY_CYCLE_START) * xScale);
        if (xTick >= x && xTick <= x + width) {
            tft.drawLine(xTick, y + height, xTick, y + height + 3, TFT_WHITE);
            tft.setCursor(xTick - 6, y + height + 5);
            tft.print(i);
        }
    }

    // Draw y-axis ticks
    const int NUM_Y_TICKS = 4;
    float tickStep = maxCurrent / NUM_Y_TICKS;
    for (int i = 0; i <= NUM_Y_TICKS; i++) {
        float currentVal = i * tickStep;
        int yTick = y + height - static_cast<int>(currentVal * yScale);
        if (yTick >= y && yTick <= y + height) {
            tft.drawLine(x - 3, yTick, x, yTick, TFT_WHITE);
            tft.setCursor(x - 25, yTick - 4);
            tft.printf("%.2f", currentVal);
        }
    }
}

// ============================================================================
// Advanced Data Processing Functions
// ============================================================================

int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    
    int low = 0;
    int high = count - 1;

    // Binary search for closest value
    while (low <= high) {
        int mid = low + (high - low) / 2;
        
        if (fabs(data[mid][0] - targetCurrent) < IRConfig::ZERO_THRESHOLD) {
            return mid;
        } else if (data[mid][0] < targetCurrent) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    // Return closest of the two candidates
    if (low >= count) return high;
    if (high < 0) return low;
    
    float diffHigh = fabs(targetCurrent - data[high][0]);
    float diffLow = fabs(data[low][0] - targetCurrent);
    
    return (diffHigh < diffLow) ? high : low;
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count >= MAX_RESISTANCE_POINTS) {
        Serial.println("Warning: Cannot insert, array full");
        return;
    }
    
    if (index < 0 || index > count) {
        Serial.printf("Warning: Invalid insert index %d (count=%d)\n", index, count);
        return;
    }
    
    // Shift elements right
    for (int i = count; i > index; --i) {
        data[i][0] = data[i - 1][0];
        data[i][1] = data[i - 1][1];
    }
    
    data[index][0] = current;
    data[index][1] = resistance;
    count++;
}

void averageDataPoints(float data[][2], int index1, int index2) {
    if (index1 < 0 || index2 < 0) {
        Serial.println("Warning: Invalid indices for averaging");
        return;
    }
    
    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index < 0 || index >= count) {
        Serial.printf("Warning: Invalid remove index %d (count=%d)\n", index, count);
        return;
    }
    
    // Shift elements left
    for (int i = index; i < count - 1; ++i) {
        data[i][0] = data[i + 1][0];
        data[i][1] = data[i + 1][1];
    }
    count--;
}

void storeOrAverageResistanceData(float current, float resistance, 
                                 float data[][2], int& count) {
    // Validate inputs
    if (!isValidResistance(resistance) || !isValidCurrent(current)) {
        return;
    }

    if (count < MAX_RESISTANCE_POINTS) {
        // Find insertion position (maintain sorted order by current)
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) {
            insertIndex++;
        }
        insertDataPoint(data, count, current, resistance, insertIndex);
    } else {
        // Array is full - need to merge with existing data
        int closestIndex = findClosestIndex(data, count, current);
        
        if (closestIndex < 0 || closestIndex >= count) {
            Serial.println("Warning: Invalid closest index in merge");
            return;
        }

        // Calculate dynamic isolation threshold
        float isolationThreshold = IRConfig::DEFAULT_ISOLATION_THRESHOLD;
        
        if (count >= 2) {
            std::vector<float> spacings;
            spacings.reserve(count - 1);
            
            for (int i = 1; i < count; ++i) {
                float spacing = data[i][0] - data[i - 1][0];
                if (spacing > 0.0f) {
                    spacings.push_back(spacing);
                }
            }

            if (!spacings.empty()) {
                float sum = 0.0f;
                for (float spacing : spacings) {
                    sum += spacing;
                }
                float meanSpacing = sum / spacings.size();
                float stdDevSpacing = standardDeviation(spacings);

                // Adaptive threshold: mean + 1.5 * std dev
                isolationThreshold = max(IRConfig::DEFAULT_ISOLATION_THRESHOLD, 
                                        meanSpacing + IRConfig::ERROR_THRESHOLD_MULTIPLIER * stdDevSpacing);
            }
        }

        // Check if closest point is isolated
        bool isIsolated = false;
        if (count > 1) {
            float distanceToPrev = (closestIndex > 0) ? 
                fabs(data[closestIndex][0] - data[closestIndex - 1][0]) : 
                isolationThreshold * 2.0f;
            float distanceToNext = (closestIndex < count - 1) ? 
                fabs(data[closestIndex][0] - data[closestIndex + 1][0]) : 
                isolationThreshold * 2.0f;

            isIsolated = (distanceToPrev >= isolationThreshold && 
                         distanceToNext >= isolationThreshold);
        }

        if (isIsolated && count >= 2) {
            // Merge neighbors and insert new point
            int index1 = (closestIndex > 0) ? closestIndex - 1 : -1;
            int index2 = (closestIndex < count - 1) ? closestIndex + 1 : -1;

            if (index1 >= 0 && index2 >= 0) {
                // Average the two neighbors
                averageDataPoints(data, index1, index2);
                removeDataPoint(data, count, index2);
                
                // Find new insertion position
                int insertIndex = 0;
                while (insertIndex < count && data[insertIndex][0] < current) {
                    insertIndex++;
                }
                insertDataPoint(data, count, current, resistance, insertIndex);
            } else {
                // Only one neighbor - average with closest point
                data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
                data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
            }
        } else {
            // Not isolated - average with closest point
            data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
            data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
        }
    }
}

void distribute_error(float data[][2], int count, 
                     float spacing_threshold, float error_threshold_multiplier) {
    if (count < IRConfig::MIN_CLUSTER_SIZE) return;

    for (int i = 0; i <= count - IRConfig::MIN_CLUSTER_SIZE; ++i) {
        for (int j = i + IRConfig::MIN_CLUSTER_SIZE - 1; j < count; ++j) {
            float clusterSpan = data[j][0] - data[i][0];
            
            if (clusterSpan <= spacing_threshold) {
                // Found a cluster
                int clusterSize = j - i + 1;
                std::vector<float> clusterResistances;
                clusterResistances.reserve(clusterSize);
                
                for (int k = i; k <= j; ++k) {
                    clusterResistances.push_back(data[k][1]);
                }

                if (clusterResistances.size() >= static_cast<size_t>(IRConfig::MIN_CLUSTER_SIZE)) {
                    // Calculate cluster statistics
                    float sum = 0.0f;
                    for (float r : clusterResistances) {
                        sum += r;
                    }
                    float average = sum / clusterResistances.size();
                    float stdDev = standardDeviation(clusterResistances);

                    // Identify and correct outliers
                    float errorThreshold = error_threshold_multiplier * stdDev;
                    int correctionCount = 0;
                    
                    for (int k = i; k <= j; ++k) {
                        if (fabs(data[k][1] - average) > errorThreshold) {
                            data[k][1] = average;
                            correctionCount++;
                        }
                    }
                    
                    if (correctionCount > 0) {
                        Serial.printf("Corrected %d outliers in cluster [%d-%d] (avg=%.4f, std=%.4f)\n", 
                                    correctionCount, i, j, average, stdDev);
                    }
                    
                    // Skip past this cluster
                    i = j;
                    break;
                }
            } else {
                // No more points within threshold
                break;
            }
        }
    }
}

// Legacy function for compatibility
void performLinearRegression(const std::vector<float>& voltages, 
                             const std::vector<float>& currents) {
    if (voltages.size() < 2 || voltages.size() != currents.size()) {
        Serial.println("Insufficient or mismatched data for regression");
        return;
    }

    Serial.println("Calculating overall IR using linear regression...");

    double sumI = 0.0, sumV = 0.0, sumII = 0.0, sumIV = 0.0;
    int n = voltages.size();
    
    for (size_t i = 0; i < currents.size(); ++i) {
        sumI += currents[i];
        sumV += voltages[i];
        sumII += currents[i] * currents[i];
        sumIV += currents[i] * voltages[i];
    }

    double denominator = (n * sumII - sumI * sumI);

    if (fabs(denominator) > IRConfig::REGRESSION_MIN_DENOMINATOR) {
        double slope = (n * sumIV - sumI * sumV) / denominator;
        double intercept = (sumV - slope * sumI) / n;

        Serial.printf("Overall IR (Linear Regression): %.4f Ω\n", fabs(slope));
        Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", intercept);
        
        // Calculate R-squared
        double meanV = sumV / n;
        double ssTotal = 0.0;
        double ssResidual = 0.0;
        
        for (size_t i = 0; i < currents.size(); ++i) {
            double vPred = slope * currents[i] + intercept;
            ssTotal += (voltages[i] - meanV) * (voltages[i] - meanV);
            ssResidual += (voltages[i] - vPred) * (voltages[i] - vPred);
        }
        
        double rSquared = (ssTotal > IRConfig::ZERO_THRESHOLD) ? 
                         (1.0 - ssResidual / ssTotal) : 0.0;
        Serial.printf("  R² = %.4f\n", rSquared);
    } else {
        Serial.println("Error: Singular matrix in regression");
    }
}

// ============================================================================
// Public API Functions
// ============================================================================

void startInternalResistanceMeasurement() {
    if (isMeasuringResistance) {
        Serial.println("Warning: Measurement already in progress");
        return;
    }
    
    isMeasuringResistance = true;
    currentIRState = IR_STATE_START;
    nextIRState = IR_STATE_IDLE;
}

void stopInternalResistanceMeasurement() {
    if (!isMeasuringResistance) {
        Serial.println("Warning: No measurement in progress");
        return;
    }
    
    Serial.println("Stopping internal resistance measurement...");
    
    // Turn off load
    analogWrite(pwmPin, 0);
    dutyCycle = 0;
    
    // Reset state
    isMeasuringResistance = false;
    currentIRState = IR_STATE_IDLE;
    nextIRState = IR_STATE_IDLE;
    
    // Print partial results if any
    if (resistanceDataCount > 0 || resistanceDataCountPairs > 0) {
        Serial.println("\n=== Partial Results ===");
        Serial.printf("Collected %d L/UL points, %d pair points\n", 
                     resistanceDataCount, resistanceDataCountPairs);
    }
}

bool isInternalResistanceMeasurementActive() {
    return isMeasuringResistance;
}

IRState getCurrentIRState() {
    return currentIRState;
}

int getResistanceDataCount() {
    return resistanceDataCount;
}

int getResistanceDataPairsCount() {
    return resistanceDataCountPairs;
}

void getResistanceDataPoint(int index, float& current, float& resistance) {
    if (index >= 0 && index < resistanceDataCount) {
        current = internalResistanceData[index][0];
        resistance = internalResistanceData[index][1];
    } else {
        current = -1.0f;
        resistance = -1.0f;
    }
}

void getResistanceDataPairsPoint(int index, float& current, float& resistance) {
    if (index >= 0 && index < resistanceDataCountPairs) {
        current = internalResistanceDataPairs[index][0];
        resistance = internalResistanceDataPairs[index][1];
    } else {
        current = -1.0f;
        resistance = -1.0f;
    }
}

void getRegressionResults(float& luSlope, float& luIntercept, 
                         float& pairsSlope, float& pairsIntercept) {
    luSlope = regressedInternalResistanceSlope;
    luIntercept = regressedInternalResistanceIntercept;
    pairsSlope = regressedInternalResistancePairsSlope;
    pairsIntercept = regressedInternalResistancePairsIntercept;
}

float getAverageInternalResistance() {
    if (consecutiveInternalResistances.empty()) {
        return -1.0f;
    }
    return calculateAverageInternalResistance(consecutiveInternalResistances);
}

// ============================================================================
// Debug and Diagnostic Functions
// ============================================================================

void printResistanceData() {
    Serial.println("\n=== Loaded/Unloaded Resistance Data ===");
    Serial.printf("Count: %d points\n", resistanceDataCount);
    Serial.println("Current(A)\tResistance(Ω)");
    
    for (int i = 0; i < resistanceDataCount; i++) {
        Serial.printf("%.3f\t\t%.4f\n", 
                     internalResistanceData[i][0], 
                     internalResistanceData[i][1]);
    }
    
    Serial.println("\n=== Pairs Resistance Data ===");
    Serial.printf("Count: %d points\n", resistanceDataCountPairs);
    Serial.println("Current(A)\tResistance(Ω)");
    
    for (int i = 0; i < resistanceDataCountPairs; i++) {
        Serial.printf("%.3f\t\t%.4f\n", 
                     internalResistanceDataPairs[i][0], 
                     internalResistanceDataPairs[i][1]);
    }
}

void printDutyCyclePairs() {
    Serial.println("\n=== Duty Cycle Pairs ===");
    Serial.printf("Count: %zu pairs\n", dutyCyclePairs.size());
    Serial.println("Pair#\tLow\tHigh");
    
    for (size_t i = 0; i < dutyCyclePairs.size(); i++) {
        Serial.printf("%zu\t%d\t%d\n", 
                     i, 
                     dutyCyclePairs[i].first, 
                     dutyCyclePairs[i].second);
    }
}

void printMeasurementProgress() {
    Serial.println("\n=== Measurement Progress ===");
    Serial.printf("State: ");
    
    switch (currentIRState) {
        case IR_STATE_IDLE:
            Serial.println("IDLE");
            break;
        case IR_STATE_START:
            Serial.println("START");
            break;
        case IR_STATE_STOP_LOAD_WAIT:
            Serial.println("STOP_LOAD_WAIT");
            break;
        case IR_STATE_GET_UNLOADED_VOLTAGE:
            Serial.println("GET_UNLOADED_VOLTAGE");
            break;
        case IR_STATE_FIND_MIN_DC:
            Serial.printf("FIND_MIN_DC (Range: %d-%d, Mid: %d)\n", 
                         findMinDcLow, findMinDcHigh, findMinDcMid);
            break;
        case IR_STATE_GENERATE_PAIRS:
            Serial.printf("GENERATE_PAIRS (Step: %d, SubStep: %d, Pair: %d)\n", 
                         pairGenerationStep, pairGenerationSubStep, pairIndex);
            break;
        case IR_STATE_MEASURE_L_UL:
            Serial.printf("MEASURE_L_UL (Pair: %d/%zu, Step: %d)\n", 
                         pairIndex, dutyCyclePairs.size(), measureStep);
            break;
        case IR_STATE_MEASURE_PAIRS:
            Serial.printf("MEASURE_PAIRS (Pair: %d/%zu, Step: %d)\n", 
                         pairIndex, dutyCyclePairs.size(), measureStep);
            break;
        case IR_STATE_GET_MEASUREMENT:
            Serial.printf("GET_MEASUREMENT (waiting for stabilization)\n");
            break;
        case IR_STATE_COMPLETE:
            Serial.println("COMPLETE");
            break;
        default:
            Serial.println("UNKNOWN");
            break;
    }
    
    Serial.printf("Active: %s\n", isMeasuringResistance ? "Yes" : "No");
    Serial.printf("Minimal DC: %d\n", minimalDutyCycle);
    Serial.printf("Current DC: %d\n", dutyCycle);
    Serial.printf("L/UL Data Points: %d\n", resistanceDataCount);
    Serial.printf("Pairs Data Points: %d\n", resistanceDataCountPairs);
    Serial.printf("Generated Pairs: %zu\n", dutyCyclePairs.size());
}

const char* getIRStateString(IRState state) {
    switch (state) {
        case IR_STATE_IDLE: return "IDLE";
        case IR_STATE_START: return "START";
        case IR_STATE_STOP_LOAD_WAIT: return "STOP_LOAD_WAIT";
        case IR_STATE_GET_UNLOADED_VOLTAGE: return "GET_UNLOADED_VOLTAGE";
        case IR_STATE_FIND_MIN_DC: return "FIND_MIN_DC";
        case IR_STATE_GENERATE_PAIRS: return "GENERATE_PAIRS";
        case IR_STATE_MEASURE_L_UL: return "MEASURE_L_UL";
        case IR_STATE_MEASURE_PAIRS: return "MEASURE_PAIRS";
        case IR_STATE_GET_MEASUREMENT: return "GET_MEASUREMENT";
        case IR_STATE_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Advanced Analysis Functions
// ============================================================================

float calculateMedianResistance(float data[][2], int count) {
    if (count == 0) return -1.0f;
    if (count == 1) return data[0][1];
    
    // Create temporary array for resistance values
    float* resistances = new float[count];
    if (resistances == nullptr) {
        Serial.println("Error: Memory allocation failed");
        return -1.0f;
    }
    
    // Copy resistance values
    for (int i = 0; i < count; i++) {
        resistances[i] = data[i][1];
    }
    
    // Simple selection sort for small arrays
    for (int i = 0; i < count - 1; i++) {
        int minIdx = i;
        for (int j = i + 1; j < count; j++) {
            if (resistances[j] < resistances[minIdx]) {
                minIdx = j;
            }
        }
        if (minIdx != i) {
            float temp = resistances[i];
            resistances[i] = resistances[minIdx];
            resistances[minIdx] = temp;
        }
    }
    
    float median;
    if (count % 2 == 0) {
        median = (resistances[count/2 - 1] + resistances[count/2]) / 2.0f;
    } else {
        median = resistances[count/2];
    }
    
    delete[] resistances;
    return median;
}

void calculateResistanceStatistics(float data[][2], int count, 
                                   float& mean, float& median, 
                                   float& stdDev, float& min, float& max) {
    if (count == 0) {
        mean = median = stdDev = min = max = -1.0f;
        return;
    }
    
    // Calculate mean and find min/max
    float sum = 0.0f;
    min = data[0][1];
    max = data[0][1];
    
    for (int i = 0; i < count; i++) {
        float r = data[i][1];
        sum += r;
        if (r < min) min = r;
        if (r > max) max = r;
    }
    
    mean = sum / count;
    
    // Calculate standard deviation
    float sqSum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = data[i][1] - mean;
        sqSum += diff * diff;
    }
    stdDev = sqrt(sqSum / count);
    
    // Calculate median
    median = calculateMedianResistance(data, count);
    
    Serial.println("\n=== Resistance Statistics ===");
    Serial.printf("Mean: %.4f Ω\n", mean);
    Serial.printf("Median: %.4f Ω\n", median);
    Serial.printf("Std Dev: %.4f Ω\n", stdDev);
    Serial.printf("Min: %.4f Ω\n", min);
    Serial.printf("Max: %.4f Ω\n", max);
    Serial.printf("Range: %.4f Ω\n", max - min);
    Serial.printf("CV: %.2f%%\n", (mean > 0.0f) ? (stdDev / mean * 100.0f) : 0.0f);
}

int removeOutliers(float data[][2], int& count, float zScoreThreshold) {
    if (count < 3) return 0; // Need at least 3 points
    
    // Calculate mean and std dev
    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += data[i][1];
    }
    float mean = sum / count;
    
    float sqSum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = data[i][1] - mean;
        sqSum += diff * diff;
    }
    float stdDev = sqrt(sqSum / count);
    
    if (stdDev < IRConfig::ZERO_THRESHOLD) {
        return 0; // No variation, no outliers
    }
    
    // Mark outliers
    bool* isOutlier = new bool[count];
    if (isOutlier == nullptr) {
        Serial.println("Error: Memory allocation failed");
        return 0;
    }
    
    int outlierCount = 0;
    for (int i = 0; i < count; i++) {
        float zScore = fabs((data[i][1] - mean) / stdDev);
        isOutlier[i] = (zScore > zScoreThreshold);
        if (isOutlier[i]) outlierCount++;
    }
    
    // Remove outliers (shift data)
    int writeIdx = 0;
    for (int readIdx = 0; readIdx < count; readIdx++) {
        if (!isOutlier[readIdx]) {
            if (writeIdx != readIdx) {
                data[writeIdx][0] = data[readIdx][0];
                data[writeIdx][1] = data[readIdx][1];
            }
            writeIdx++;
        }
    }
    
    delete[] isOutlier;
    
    count = writeIdx;
    return outlierCount;
}

void smoothResistanceData(float data[][2], int count, int windowSize) {
    if (count < windowSize || windowSize < 2) return;
    
    // Create temporary array for smoothed values
    float* smoothed = new float[count];
    if (smoothed == nullptr) {
        Serial.println("Error: Memory allocation failed");
        return;
    }
    
    // Apply moving average
    int halfWindow = windowSize / 2;
    
    for (int i = 0; i < count; i++) {
        float sum = 0.0f;
        int validCount = 0;
        
        int start = max(0, i - halfWindow);
        int end = min(count - 1, i + halfWindow);
        
        for (int j = start; j <= end; j++) {
            sum += data[j][1];
            validCount++;
        }
        
        smoothed[i] = sum / validCount;
    }
    
    // Copy smoothed values back
    for (int i = 0; i < count; i++) {
        data[i][1] = smoothed[i];
    }
    
    delete[] smoothed;
    
    Serial.printf("Applied moving average smoothing (window=%d)\n", windowSize);
}

// ============================================================================
// Configuration Functions
// ============================================================================

void setMeasurementConfiguration(int minDC, int maxDC, float minCurrentThreshold, 
                                 int stabilizationMs, int unloadedDelayMs) {
    // Validate inputs
    if (minDC < 0 || minDC > 255 || maxDC < 0 || maxDC > 255 || minDC >= maxDC) {
        Serial.println("Error: Invalid duty cycle range");
        return;
    }
    
    if (minCurrentThreshold < 0.0f || minCurrentThreshold > 10.0f) {
        Serial.println("Error: Invalid current threshold");
        return;
    }
    
    if (stabilizationMs < 0 || unloadedDelayMs < 0) {
        Serial.println("Error: Invalid delay values");
        return;
    }
    
    // Note: These would need to be stored in global variables if we want
    // to make them configurable. For now, they're constants in definitions.h
    Serial.println("Configuration update requested (requires code modification)");
    Serial.printf("  Min DC: %d\n", minDC);
    Serial.printf("  Max DC: %d\n", maxDC);
    Serial.printf("  Min Current: %.3f A\n", minCurrentThreshold);
    Serial.printf("  Stabilization: %d ms\n", stabilizationMs);
    Serial.printf("  Unloaded Delay: %d ms\n", unloadedDelayMs);
}

// ============================================================================
// End of File
// ============================================================================
