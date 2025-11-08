
#include "internal_resistance.h"
#include "definitions.h"
#include <algorithm> // for nth_element


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

// Helper function to initiate measurement
void getSingleMeasurement(int dc, IRState nextState) {
    dutyCycle = dc;
    analogWrite(pwmPin, dutyCycle);
    irStateChangeTime = millis();
    nextIRState = nextState;
    currentIRState = IR_STATE_GET_MEASUREMENT;
}

// Reset all state variables for new measurement
void resetMeasurementState() {
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    voltagesLoaded.clear();
    currentsLoaded.clear();
    ir_dutyCycles.clear();
    consecutiveInternalResistances.clear();
    dutyCyclePairs.clear();

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
            // Do nothing - waiting for start command
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
                Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

                // Initialize binary search parameters
                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;

                Serial.println("Finding minimal duty cycle for measurable current using binary search...");
//                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);
                tft.drawString("Finding Min Duty Cycle", 10, 10);

                currentIRState = IR_STATE_FIND_MIN_DC;
                // Start first measurement
                findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                getSingleMeasurement(findMinDcMid, IR_STATE_FIND_MIN_DC);
            }
            break;

        case IR_STATE_FIND_MIN_DC:
            // Process the measurement result
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
                    //tft.fillScreen(TFT_BLACK);
                    tft.setTextColor(TFT_WHITE);
                    tft.setTextSize(2);

                    if (minimalDutyCycle > 0) {
                        Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
                        tft.printf("Minimal Duty Cycle:\n%d%%\n", minimalDutyCycle);
                        currentIRState = IR_STATE_GENERATE_PAIRS;
                    } else {
                        Serial.println("Warning: Could not find duty cycle producing measurable current.");
                        tft.println("Warning: No measurable\ncurrent found.");
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

void handleGeneratePairs() {
    switch (pairGenerationStep) {
        case 0:
            Serial.println("Generating duty cycle pairs...");
            if (minimalDutyCycle == 0) {
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
            Serial.printf("Min current at DC %d: %.3f A\n", minimalDutyCycle, minCurrent);
            getSingleMeasurement(MAX_DUTY_CYCLE, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 2;
            break;

        case 2:
            maxCurrent = currentMeasurement.current;
            Serial.printf("Max current at DC %d: %.3f A\n", MAX_DUTY_CYCLE, maxCurrent);

            if (maxCurrent <= minCurrent) {
                Serial.println("Error: Invalid current range");
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }

            lowDc = minimalDutyCycle;
            previousHighDc = MAX_DUTY_CYCLE;
            pairIndex = 0;
            pairGenerationStep = 3;
            pairGenerationSubStep = 0;
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
    if (pairIndex >= dutyCyclePairs.size()) {
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
                if (loadedCurrent > 0.01f) {
                    float internalResistance = (currentMeasurement.voltage - voltagesLoaded.back()) / loadedCurrent;
                    storeResistanceData(loadedCurrent, fabs(internalResistance),
                                      internalResistanceData, resistanceDataCount);
                    Serial.printf("L/UL Point %d: I=%.3fA, R=%.4fΩ\n",
                                pairIndex, loadedCurrent, fabs(internalResistance));
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void handleMeasurePairs() {
    if (pairIndex >= dutyCyclePairs.size()) {
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
                    consecutiveInternalResistances.push_back(fabs(internalResistance));
                    storeResistanceData(currentMeasurement.current, fabs(internalResistance),
                                      internalResistanceDataPairs, resistanceDataCountPairs);
                    Serial.printf("Pair %d: ΔI=%.3fA, R=%.4fΩ\n",
                                pairIndex, currentDiff, fabs(internalResistance));
                } else {
                    consecutiveInternalResistances.push_back(-1.0f);
                    Serial.printf("Pair %d: Insufficient current difference (%.3fA)\n",
                                pairIndex, currentDiff);
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void completeResistanceMeasurement() {
    Serial.println("\n=== Measurement Complete ===");

    // Sort data by current
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    // Derive spacing_threshold from overall span or mean spacing:
    auto compute_spacing_threshold = [](float data[][2], int n) -> float {
        if (n < 2) return 0.05f;
        // use typical spacing:
        float minX = data[0][0], maxX = data[n-1][0];
        float avgSpacing = (maxX - minX) / max(1, n-1);
        return max(0.02f, avgSpacing * 1.5f); // tune multiplier
    };

    float spacing_thresh = compute_spacing_threshold(internalResistanceData, resistanceDataCount);
    distribute_error(internalResistanceData, resistanceDataCount, spacing_thresh, 1.5f);

    float spacing_thresh_pairs = compute_spacing_threshold(internalResistanceDataPairs, resistanceDataCountPairs);
    distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, spacing_thresh_pairs, 1.5f);

    // Perform regression on loaded/unloaded data
    if (resistanceDataCount >= 2) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount,
                                    regressedInternalResistanceSlope,
                                    regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed IR (Loaded/Unloaded): Slope=%.4f, Intercept=%.4f\n",
                         regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
    } else {
        Serial.println("Insufficient data for L/UL regression");
    }

    // Perform regression on paired data
    if (resistanceDataCountPairs >= 2) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                    regressedInternalResistancePairsSlope,
                                    regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed IR (Pairs): Slope=%.4f, Intercept=%.4f\n",
                         regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.println("Insufficient data for pairs regression");
    }

    Serial.printf("\nCollected %d L/UL points, %d pair points\n",
                 resistanceDataCount, resistanceDataCountPairs);

    isMeasuringResistance = false;
    currentIRState = IR_STATE_IDLE;
}

void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        bool swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                // Swap both values
                float temp0 = data[j][0];
                float temp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0;
                data[j + 1][1] = temp1;
                swapped = true;
            }
        }
        if (!swapped) break; // Early exit if already sorted
    }
}

void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count >= MAX_RESISTANCE_POINTS) {
        Serial.println("Warning: Resistance data array full");
        return;
    }

    if (resistance > MIN_VALID_RESISTANCE && resistance < 1000.0f) { // Add upper bound check
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
    const uint16_t GREY = TFT_DARKGREY;
    const int BAR_Y = 10;
    const int BAR_HEIGHT = 10;
    const int BAR_START_X = 30;
    const int BAR_END_X = SCREEN_WIDTH - 2;
    const int BAR_WIDTH = BAR_END_X - BAR_START_X + 1;

    // Draw background and border
    tft.fillRect(BAR_START_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT, GREY);
    tft.drawRect(BAR_START_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT, TFT_WHITE);

    int range = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
    if (range <= 0) range = 1; // Prevent division by zero

    // Map duty cycles to screen coordinates
    int low_x = map(low, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X);
    int high_x = map(high, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X);
    int mid_x = map(mid, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, BAR_START_X, BAR_END_X);

    // Clamp to bar bounds
    low_x = constrain(low_x, BAR_START_X, BAR_END_X);
    high_x = constrain(high_x, BAR_START_X, BAR_END_X);
    mid_x = constrain(mid_x, BAR_START_X, BAR_END_X);

    // Draw range markers
    tft.drawLine(low_x, BAR_Y - 5, low_x, BAR_Y + BAR_HEIGHT + 5, TFT_WHITE);
    tft.drawLine(high_x, BAR_Y - 5, high_x, BAR_Y + BAR_HEIGHT + 5, TFT_WHITE);

    // Draw current position indicator
    uint16_t mid_color = (current >= threshold) ? TFT_GREEN : TFT_RED;
    tft.fillCircle(mid_x, BAR_Y + BAR_HEIGHT / 2, 6, mid_color);

    // Display text info
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, BAR_Y + BAR_HEIGHT + 15);
    tft.printf("L:%d%% H:%d%% M:%d%%", low, high, mid);
    tft.setCursor(10, BAR_Y + BAR_HEIGHT + 30);
    tft.printf("I:%.3fA Thr:%.3fA %s", current, threshold,
              (current >= threshold) ? "OK" : "LOW");
}

void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent,
               int x, int y, int width, int height) {
    if (numPoints <= 1 || width <= 0 || height <= 0) return;

    // Draw frame
    tft.fillRect(x, y, width, height, TFT_BLACK);
    tft.drawRect(x, y, width, height, TFT_WHITE);

    // Calculate scales
    int dcRange = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
    if (dcRange <= 0) dcRange = 1;

    float xScale = (float)width / dcRange;
    float yScale = (maxCurrent > 0) ? (float)height / maxCurrent : 0.0f;

    // Draw data points and lines
    for (int i = 0; i < numPoints - 1; i++) {
        int x1 = x + (dutyCycles[i] - MIN_DUTY_CYCLE_START) * xScale;
        int y1 = y + height - currents[i] * yScale;
        int x2 = x + (dutyCycles[i + 1] - MIN_DUTY_CYCLE_START) * xScale;
        int y2 = y + height - currents[i + 1] * yScale;

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
    tft.setCursor(x + width / 2 - 30, y + height + 5);
    tft.print("DC (%)");
    tft.setCursor(x - 25, y + height / 2);
    tft.print("I(A)");

    // Draw x-axis ticks
    int tickInterval = max(1, dcRange / 5);
    for (int i = MIN_DUTY_CYCLE_START; i <= MAX_DUTY_CYCLE; i += tickInterval) {
        int xTick = x + (i - MIN_DUTY_CYCLE_START) * xScale;
        if (xTick >= x && xTick <= x + width) {
            tft.drawLine(xTick, y + height, xTick, y + height + 3, TFT_WHITE);
            tft.setCursor(xTick - 6, y + height + 5);
            tft.print(i);
        }
    }

    // Draw y-axis ticks
    if (maxCurrent > 0) {
        float tickStep = maxCurrent / 4;
        for (int i = 0; i <= 4; i++) {
            float currentVal = i * tickStep;
            int yTick = y + height - currentVal * yScale;
            if (yTick >= y && yTick <= y + height) {
                tft.drawLine(x - 3, yTick, x, yTick, TFT_WHITE);
                tft.setCursor(x - 25, yTick - 4);
                tft.printf("%.2f", currentVal);
            }
        }
    }
}

float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    float sum = 0.0f;
    int count = 0;

    for (float rint : resistances) {
        if (rint > MIN_VALID_RESISTANCE && rint < 1000.0f) { // Add bounds check
            sum += rint;
            count++;
        }
    }

    if (count > 0) {
        float average = sum / count;
        Serial.printf("\nAverage Internal Resistance: %.4f Ω (%d valid points)\n", average, count);
        return average;
    } else {
        Serial.println("\nNo valid internal resistance measurements.");
        return -1.0f;
    }
}

void performLinearRegression(const std::vector<float>& voltages, const std::vector<float>& currents) {
    if (voltages.size() < 2 || voltages.size() != currents.size()) {
        Serial.println("Insufficient or mismatched data for regression");
        return;
    }

    Serial.println("Calculating overall internal resistance using linear regression...");

    double sumI = 0.0, sumV = 0.0, sumII = 0.0, sumIV = 0.0;
    int n = voltages.size();

    for (size_t i = 0; i < currents.size(); ++i) {
        sumI += currents[i];
        sumV += voltages[i];
        sumII += currents[i] * currents[i];
        sumIV += currents[i] * voltages[i];
    }

    double denominator = (n * sumII - sumI * sumI);

    if (fabs(denominator) > 1e-9) {
        double slope = (n * sumIV - sumI * sumV) / denominator;
        double intercept = (sumV - slope * sumI) / n;

        Serial.printf("Overall IR (Linear Regression): %.4f Ω\n", fabs(slope));
        Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", intercept);
    } else {
        Serial.println("Error: Singular matrix in regression (division by zero)");
    }
}

int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;

    int low = 0;
    int high = count - 1;

    while (low <= high) {
        int mid = low + (high - low) / 2;

        if (fabs(data[mid][0] - targetCurrent) < 1e-6f) {
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

    return (fabs(targetCurrent - data[high][0]) < fabs(data[low][0] - targetCurrent)) ? high : low;
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count >= MAX_RESISTANCE_POINTS) {
        Serial.println("Warning: Cannot insert, array full");
        return;
    }

    if (index < 0 || index > count) {
        Serial.println("Warning: Invalid insert index");
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
    if (index1 < 0 || index2 < 0) return;

    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index < 0 || index >= count) {
        Serial.println("Warning: Invalid remove index");
        return;
    }

    // Shift elements left
    for (int i = index; i < count - 1; ++i) {
        data[i][0] = data[i + 1][0];
        data[i][1] = data[i + 1][1];
    }
    count--;
}

float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;

    float sum = 0.0f;
    for (float val : data) {
        sum += val;
    }
    float mean = sum / data.size();

    float sqSum = 0.0f;
    for (float val : data) {
        float diff = val - mean;
        sqSum += diff * diff;
    }

    return sqrt(sqSum / data.size());
}


void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    // Validate input
    if (resistance <= MIN_VALID_RESISTANCE || resistance >= 1000.0f) return;
    if (current < 0.0f) {
        Serial.println("Warning: Negative current value");
        return;
    }

    if (count < MAX_RESISTANCE_POINTS) {
        // Find insertion position (maintain sorted order)
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
        insertDataPoint(data, count, current, resistance, insertIndex);
        return;
    }

    // If full: find where the point would go
    int closestIndex = findClosestIndex(data, count, current);
    if (closestIndex < 0 || closestIndex >= count) {
        Serial.println("Warning: Invalid closest index");
        return;
    }

    // If very close to an existing point (tolerance), do weighted average
    const float CLOSE_TOLERANCE = 1e-3f * max(1.0f, current); // relative tolerance
    if (fabs(data[closestIndex][0] - current) <= CLOSE_TOLERANCE) {
        // Exponential smoothing with alpha (new sample weight)
        const float alpha = 0.5f; // tune: 0.5 gives equal weight to new sample
        data[closestIndex][1] = alpha * resistance + (1.0f - alpha) * data[closestIndex][1];
        data[closestIndex][0] = alpha * current + (1.0f - alpha) * data[closestIndex][0];
        return;
    }

    // Otherwise, evict the most isolated point (largest neighbor gap)
    int evictIndex = 0;
    float maxGap = -1.0f;
    for (int i = 0; i < count; ++i) {
        float leftGap = (i > 0) ? data[i][0] - data[i-1][0] : 0.0f;
        float rightGap = (i < count-1) ? data[i+1][0] - data[i][0] : 0.0f;
        float localGap = max(leftGap, rightGap);
        if (localGap > maxGap) {
            maxGap = localGap;
            evictIndex = i;
        }
    }

    // Remove the most isolated point and insert the new one
    removeDataPoint(data, count, evictIndex);

    // Insert new in sorted order
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
        // need lower median too
        auto it = std::max_element(v.begin(), v.begin() + mid);
        med = (med + *it) / 2.0f;
    }
    return med;
}

void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return;

    for (int i = 0; i <= count - 4; ++i) {
        for (int j = i + 3; j < count; ++j) {
            float clusterSpan = data[j][0] - data[i][0];
            if (clusterSpan <= spacing_threshold) {
                // Collect cluster resistances
                std::vector<float> clusterResistances;
                clusterResistances.reserve(j - i + 1);
                for (int k = i; k <= j; ++k) clusterResistances.push_back(data[k][1]);

                if (clusterResistances.size() >= 4) {
                    // median -> robust center
                    float median = computeMedian(clusterResistances);
                    // compute stddev relative to median (robust-ish)
                    float sumSq = 0.0f;
                    for (float r : clusterResistances) {
                        float d = r - median;
                        sumSq += d * d;
                    }
                    float stdDev = sqrt(sumSq / clusterResistances.size());

                    float errorThreshold = error_threshold_multiplier * stdDev;
                    int correctionCount = 0;

                    // Instead of hard replace, blend the outlier toward median.
                    // alpha controls how strongly we pull the outlier back (0..1).
                    const float alpha = 0.6f; // 1.0 -> full replace (old behavior); 0.6 is gentler
                    for (int k = i; k <= j; ++k) {
                        float diff = fabs(data[k][1] - median);
                        if (diff > errorThreshold && stdDev > 1e-9f) {
                            data[k][1] = alpha * median + (1.0f - alpha) * data[k][1];
                            correctionCount++;
                        }
                    }

                    if (correctionCount > 0) {
                        Serial.printf("Corrected %d outliers in cluster [%d-%d]\n", correctionCount, i, j);
                    }

                    // Advance past this cluster
                    i = j;
                    break;
                }
            } else {
                break;
            }
        }
    }
}



bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) {
        Serial.println("Error: Insufficient data points for linear regression");
        return false;
    }

    // Calculate sums for regression
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

    if (fabs(denominator) < 1e-6f) {
        Serial.println("Error: Singular matrix in regression (near-zero denominator)");
        return false;
    }

    // Calculate slope and intercept
    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / n;

    // Calculate R-squared for quality assessment
    float meanY = sumY / n;
    float ssTotal = 0.0f;
    float ssResidual = 0.0f;

    for (int i = 0; i < count; ++i) {
        float x = data[i][0];
        float y = data[i][1];
        float yPred = slope * x + intercept;

        ssTotal += (y - meanY) * (y - meanY);
        ssResidual += (y - yPred) * (y - yPred);
    }

    float rSquared = (ssTotal > 1e-6f) ? (1.0f - ssResidual / ssTotal) : 0.0f;
    Serial.printf("Regression R² = %.4f\n", rSquared);

    return true;
}
