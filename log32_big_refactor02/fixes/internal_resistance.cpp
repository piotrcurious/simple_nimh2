// Improved/internal_resistance_improved.cpp
#include "internal_resistance.h"
#include "definitions.h"

// External functions from your existing codebase
extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
extern void processThermistorData(const MeasurementData& data, const String& measurementType);

// === configuration / platform abstraction ===
// If your ESP32 code uses ledcWrite, define USE_LEDC and set channel/freq/resolution in definitions.h
#ifndef USE_LEDC
  #define USE_LEDC 1
#endif

#if USE_LEDC
  // ensure pwmChannel is defined in definitions.h or set here
  #ifndef PWM_CHANNEL
    #define PWM_CHANNEL 0
  #endif
#endif

// === state machine variables (kept similar to your original design) ===
IRState currentIRState = IR_STATE_IDLE;
unsigned long irStateChangeTime = 0UL;
MeasurementData currentMeasurement;
IRState nextIRState = IR_STATE_IDLE;

// core control variables
int dutyCycle = 0; // global duty from definitions.h previously assumed
int irDutyCycle = 0;

// find-min binary search variables (improved)
int minDutyCycle = 0;
int findMinDcLow = 0;
int findMinDcHigh = 0;
int findMinDcMid = 0;
int minimalDutyCycle = 0;
int findMinSubStep = 0; // 0 = init/issue measurement, 1 = waiting/handle result

// pair-generation variables
std::vector<std::pair<int,int>> dutyCyclePairs;
int pairIndex = 0;
float minCurrent = 0.0f;
float maxCurrent = 0.0f;
int lowDc = 0;
int previousHighDc = 0;
int pairGenerationStep = 0;
int pairGenerationSubStep = 0;
int lowBound = 0;
int highBound = 0;
int bestHighDc = -1;
float minCurrentDifference = 1e9f;

// measurement bookkeeping
int measureStep = 0;
std::vector<float> voltagesLoaded;
std::vector<float> currentsLoaded;
std::vector<float> ir_dutyCycles;
std::vector<float> consecutiveInternalResistances;

float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;

float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

bool isMeasuringResistance = false;

// Safety helpers
inline unsigned long nowMillis() { return millis(); }
inline bool timeElapsed(unsigned long start, unsigned long interval) {
    // safe millis compare
    return ((unsigned long)(millis() - start) >= (unsigned long)interval);
}

// Platform-agnostic PWM set
static void setPwmDuty(int dc) {
    if (dc < 0) dc = 0;
    if (dc > MAX_DUTY_CYCLE) dc = MAX_DUTY_CYCLE;
    dutyCycle = dc;
#if USE_LEDC
    // Chosen to use duty range 0..255 or 0..2^resolution-1 depending on LEDC resolution
    // Ensure LEDC channel and resolution are set elsewhere.
    ledcWrite(PWM_CHANNEL, dutyCycle);
#else
    analogWrite(pwmPin, dutyCycle);
#endif
}

// Small utility to guard push to vectors
template<typename T>
static void safePushBack(std::vector<T>& v, const T& val, size_t maxSize = 1000000) {
    (void)maxSize;
    v.push_back(val);
}

// --- Improved state machine stepper ---
void measureInternalResistanceStep() {
    unsigned long now = nowMillis();

    switch (currentIRState) {
        case IR_STATE_IDLE:
            // idle, nothing to do
            break;

        case IR_STATE_START:
            Serial.println("Starting improved internal resistance measurement...");
            resistanceDataCount = 0;
            resistanceDataCountPairs = 0;
            voltagesLoaded.clear();
            currentsLoaded.clear();
            ir_dutyCycles.clear();
            consecutiveInternalResistances.clear();
            dutyCyclePairs.clear();
            pairIndex = 0;
            findMinDcLow = 0;
            findMinDcHigh = 0;
            minimalDutyCycle = 0;
            findMinSubStep = 0;
            pairGenerationStep = 0;
            pairGenerationSubStep = 0;
            measureStep = 0;
            isMeasuringResistance = true;
            currentIRState = IR_STATE_STOP_LOAD_WAIT;
            irStateChangeTime = now;
            break;

        case IR_STATE_STOP_LOAD_WAIT:
            // Ensure load is turned off and battery is unloaded for a short time
            setPwmDuty(0);
            if (timeElapsed(irStateChangeTime, UNLOADED_VOLTAGE_DELAY_MS)) {
                currentIRState = IR_STATE_GET_UNLOADED_VOLTAGE;
                irStateChangeTime = now;
            }
            break;

        case IR_STATE_GET_UNLOADED_VOLTAGE: {
            MeasurementData initialUnloaded;
            getThermistorReadings(initialUnloaded.temp1, initialUnloaded.temp2, initialUnloaded.tempDiff,
                                  initialUnloaded.t1_millivolts, initialUnloaded.voltage, initialUnloaded.current);
            Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);
            // proceed to find minimal DC where current measurable
            currentIRState = IR_STATE_FIND_MIN_DC;
            findMinSubStep = 0;
            break;
        }

        case IR_STATE_FIND_MIN_DC:
        {
            // Initialize binary search only once
            if (findMinSubStep == 0) {
                Serial.println("Finding minimal duty cycle for measurable current using binary search...");
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);
                tft.drawString("Finding Min Duty Cycle", 10, 10);
                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;
                findMinSubStep = 1;
            }

            if (findMinSubStep == 1) {
                if (findMinDcLow <= findMinDcHigh) {
                    findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                    // issue measurement: set duty and switch into GET_MEASUREMENT state, returning later
                    setPwmDuty(findMinDcMid);
                    irStateChangeTime = now;
                    nextIRState = IR_STATE_FIND_MIN_DC;    // after measurement, come back here
                    currentIRState = IR_STATE_GET_MEASUREMENT;
                    // do not change findMinSubStep here - wait for measurement to complete
                } else {
                    // finished binary search
                    tft.fillScreen(TFT_BLACK);
                    tft.setTextColor(TFT_WHITE);
                    tft.setTextSize(2);
                    if (minimalDutyCycle > 0) {
                        Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
                        tft.printf("Minimal Duty Cycle Found:\n%d%%\n", minimalDutyCycle);
                        minDutyCycle = minimalDutyCycle;
                        currentIRState = IR_STATE_GENERATE_PAIRS;
                        pairGenerationStep = 0;
                    } else {
                        Serial.println("Warning: Could not find a duty cycle producing measurable current.");
                        tft.println("Warning: Could not find\nmeasurable current.");
                        currentIRState = IR_STATE_IDLE;
                        isMeasuringResistance = false;
                    }
                }
            }
            break;
        }

        case IR_STATE_GENERATE_PAIRS:
            // generate pairs systematically across current range
            if (pairGenerationStep == 0) {
                Serial.println("Generating duty cycle pairs...");
                if (minDutyCycle == 0) {
                    Serial.println("minDutyCycle is 0 - aborting pair generation");
                    currentIRState = IR_STATE_IDLE;
                    isMeasuringResistance = false;
                    break;
                }
                // measure at minDutyCycle to obtain minCurrent
                setPwmDuty(minDutyCycle);
                irStateChangeTime = now;
                nextIRState = IR_STATE_GENERATE_PAIRS;
                currentIRState = IR_STATE_GET_MEASUREMENT;
                pairGenerationStep = 1;
            } else if (pairGenerationStep == 1) {
                // we return here after measurement filled currentMeasurement
                minCurrent = currentMeasurement.current;
                // measure at max duty
                setPwmDuty(MAX_DUTY_CYCLE);
                irStateChangeTime = now;
                nextIRState = IR_STATE_GENERATE_PAIRS;
                currentIRState = IR_STATE_GET_MEASUREMENT;
                pairGenerationStep = 2;
            } else if (pairGenerationStep == 2) {
                // after max-duty measurement
                maxCurrent = currentMeasurement.current;
                lowDc = minDutyCycle;
                previousHighDc = MAX_DUTY_CYCLE;
                pairIndex = 0;
                pairGenerationStep = 3;
                pairGenerationSubStep = 0;
                Serial.printf("minCurrent=%.4f, maxCurrent=%.4f\n", minCurrent, maxCurrent);
            } else if (pairGenerationStep == 3) {
                // iterative generation of pairs by targeting uniform current levels
                size_t halfPoints = MAX_RESISTANCE_POINTS / 2;
                if ((size_t)pairIndex < halfPoints) {
                    if (pairGenerationSubStep == 0) {
                        // prepare search window for this pair
                        float targetHighCurrent = maxCurrent - (pairIndex * (maxCurrent - minCurrent) / (float)halfPoints);
                        bestHighDc = -1;
                        minCurrentDifference = 1e9f;
                        lowBound = minDutyCycle;
                        highBound = previousHighDc;
                        pairGenerationSubStep = 1;
                    } else if (pairGenerationSubStep == 1) {
                        if (lowBound <= highBound) {
                            int midDc = lowBound + (highBound - lowBound) / 2;
                            // issue measurement at midDc
                            setPwmDuty(midDc);
                            irStateChangeTime = now;
                            nextIRState = IR_STATE_GENERATE_PAIRS;
                            currentIRState = IR_STATE_GET_MEASUREMENT;
                            pairGenerationSubStep = 2;
                        } else {
                            // choose best found or fallback
                            int currentHighDc = (bestHighDc != -1) ? bestHighDc : previousHighDc;
                            if (currentHighDc < minDutyCycle) currentHighDc = minDutyCycle;
                            dutyCyclePairs.push_back({lowDc, currentHighDc});
                            previousHighDc = currentHighDc - 1;
                            pairIndex++;
                            pairGenerationSubStep = 0;
                        }
                    } else if (pairGenerationSubStep == 2) {
                        // measurement has been performed; evaluate
                        float targetHighCurrent = maxCurrent - (pairIndex * (maxCurrent - minCurrent) / (float)halfPoints);
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
                } else {
                    // finished generating dutyCyclePairs
                    currentIRState = IR_STATE_MEASURE_L_UL;
                    pairIndex = 0;
                    measureStep = 0;
                }
            }
            break;

        case IR_STATE_MEASURE_L_UL:
            // For each pair entry we measure high-DC loaded voltage & then 0% unloaded voltage
            if (pairIndex < (int)dutyCyclePairs.size()) {
                if (measureStep == 0) {
                    int dcHigh = dutyCyclePairs[pairIndex].second;
                    setPwmDuty(dcHigh);
                    irStateChangeTime = now;
                    nextIRState = IR_STATE_MEASURE_L_UL;
                    currentIRState = IR_STATE_GET_MEASUREMENT;
                    measureStep = 1;
                } else if (measureStep == 1) {
                    // store loaded measurement
                    voltagesLoaded.push_back(currentMeasurement.voltage);
                    currentsLoaded.push_back(currentMeasurement.current);
                    ir_dutyCycles.push_back(static_cast<float>(currentMeasurement.dutyCycle));
                    // measure unloaded (0%)
                    setPwmDuty(0);
                    irStateChangeTime = now;
                    nextIRState = IR_STATE_MEASURE_L_UL;
                    currentIRState = IR_STATE_GET_MEASUREMENT;
                    measureStep = 2;
                } else if (measureStep == 2) {
                    // now currentMeasurement is the unloaded reading (should be small)
                    float loadedI = (currentsLoaded.empty() ? 0.0f : currentsLoaded.back());
                    float loadedV = (voltagesLoaded.empty() ? 0.0f : voltagesLoaded.back());
                    if (loadedI > 0.0001f) {
                        float internalResistance = (currentMeasurement.voltage - loadedV) / loadedI;
                        storeResistanceData(loadedI, std::fabs(internalResistance), internalResistanceData, resistanceDataCount);
                    } else {
                        // keep an invalid marker or skip — storeResistanceData currently ignores <= MIN_VALID_RESISTANCE
                        storeResistanceData(loadedI, -1.0f, internalResistanceData, resistanceDataCount);
                    }
                    pairIndex++;
                    measureStep = 0;
                }
            } else {
                // done loaded/unloaded sweep, move to successive-pair measurements
                currentIRState = IR_STATE_MEASURE_PAIRS;
                pairIndex = 0;
                measureStep = 0;
            }
            break;

        case IR_STATE_MEASURE_PAIRS:
            // For each pair: measure low DC then high DC and compute internal resistance between the two load points
            if (pairIndex < (int)dutyCyclePairs.size()) {
                if (measureStep == 0) {
                    int dcLow = dutyCyclePairs[pairIndex].first;
                    setPwmDuty(dcLow);
                    irStateChangeTime = now;
                    nextIRState = IR_STATE_MEASURE_PAIRS;
                    currentIRState = IR_STATE_GET_MEASUREMENT;
                    measureStep = 1;
                } else if (measureStep == 1) {
                    // low-point measured and stored in currentMeasurement
                    voltagesLoaded.push_back(currentMeasurement.voltage);
                    currentsLoaded.push_back(currentMeasurement.current);
                    int dcHigh = dutyCyclePairs[pairIndex].second;
                    setPwmDuty(dcHigh);
                    irStateChangeTime = now;
                    nextIRState = IR_STATE_MEASURE_PAIRS;
                    currentIRState = IR_STATE_GET_MEASUREMENT;
                    measureStep = 2;
                } else if (measureStep == 2) {
                    // high-point measurement available in currentMeasurement
                    float lowI = currentsLoaded.back();
                    float lowV = voltagesLoaded.back();
                    float highI = currentMeasurement.current;
                    float highV = currentMeasurement.voltage;

                    if (highI > lowI + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistanceConsecutive = (lowV - highV) / (highI - lowI);
                        consecutiveInternalResistances.push_back(std::fabs(internalResistanceConsecutive));
                        storeResistanceData(highI, std::fabs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
                    } else {
                        consecutiveInternalResistances.push_back(-1.0f);
                        storeResistanceData(highI, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                    pairIndex++;
                    measureStep = 0;
                }
            } else {
                currentIRState = IR_STATE_COMPLETE;
            }
            break;

        case IR_STATE_GET_MEASUREMENT:
            // Wait STABILIZATION_DELAY_MS after setting duty before reading sensors
            if (timeElapsed(irStateChangeTime, STABILIZATION_DELAY_MS)) {
                getThermistorReadings(currentMeasurement.temp1, currentMeasurement.temp2, currentMeasurement.tempDiff,
                                      currentMeasurement.t1_millivolts, currentMeasurement.voltage, currentMeasurement.current);
                currentMeasurement.dutyCycle = dutyCycle;
                currentMeasurement.timestamp = millis();
                // optional: processThermistorData(currentMeasurement, "measurement");
                // return to next state
                currentIRState = nextIRState;
            }
            break;

        case IR_STATE_COMPLETE:
            // final processing: sort, corrective steps, regression
            bubbleSort(internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

            if (resistanceDataCount >= 2) {
                if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                } else {
                    Serial.println("Linear regression failed for Loaded/Unloaded data.");
                }
            } else {
                Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
            }

            if (resistanceDataCountPairs >= 2) {
                if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                } else {
                    Serial.println("Linear regression failed for pair data.");
                }
            } else {
                Serial.println("Not enough data points for linear regression of paired resistance.");
            }

            Serial.printf("Internal resistance measurement complete. %d loaded/unloaded points, %d pair points collected.\n", resistanceDataCount, resistanceDataCountPairs);
            isMeasuringResistance = false;
            currentIRState = IR_STATE_IDLE;
            break;

        default:
            currentIRState = IR_STATE_IDLE;
            break;
    }
}

// === utility functions with defensive checks ===

void bubbleSort(float data[][2], int n) {
    if (n <= 1) return;
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                float tmp0 = data[j][0];
                float tmp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = tmp0;
                data[j + 1][1] = tmp1;
            }
        }
    }
}

void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    // Resistances <= MIN_VALID_RESISTANCE are skipped to avoid junk
    if (count < 0) count = 0;
    if (count >= MAX_RESISTANCE_POINTS) return;
    if (resistance <= MIN_VALID_RESISTANCE) return;
    dataArray[count][0] = current;
    dataArray[count][1] = resistance;
    count++;
}

void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
    uint16_t GREY = TFT_DARKGREY;
    const int DUTY_CYCLE_BAR_Y = 10;
    const int DUTY_CYCLE_BAR_HEIGHT = 10;
    const int DUTY_CYCLE_BAR_START_X = 30;
    const int DUTY_CYCLE_BAR_END_X = SCREEN_WIDTH - 2;

    tft.fillRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y,
                 DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1,
                 DUTY_CYCLE_BAR_HEIGHT, GREY);
    tft.drawRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y,
                 DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1,
                 DUTY_CYCLE_BAR_HEIGHT, TFT_WHITE);

    int low_x = map(constrain(low, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
    int high_x = map(constrain(high, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
    int mid_x = map(constrain(mid, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE), MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);

    tft.drawLine(low_x, DUTY_CYCLE_BAR_Y - 10, low_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, TFT_WHITE);
    tft.drawLine(high_x, DUTY_CYCLE_BAR_Y - 10, high_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, TFT_WHITE);

    uint16_t mid_color = (current >= threshold) ? TFT_GREEN : TFT_RED;
    tft.fillCircle(mid_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT / 2, 8, mid_color);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.printf("Low: %d%%, High: %d%%, Mid: %d%%\n", low, high, mid);
    tft.printf("Current: %.3f A (Threshold: %.3f A)\n", current, threshold);
    tft.printf("Measurable: %s\n", (current >= threshold) ? "Yes" : "No");
}

void drawGraph(const std::vector<int>& dutyCyclesVec, const std::vector<float>& currentsVec, float maxCurrent, int x, int y, int width, int height) {
    int numPoints = std::min(dutyCyclesVec.size(), currentsVec.size());
    if (numPoints <= 1) return;

    tft.fillRect(x, y, width, height, TFT_BLACK);
    tft.drawRect(x, y, width, height, TFT_WHITE);

    float xScale = (float)width / (float)(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START);
    float yScale = (height > 0 && maxCurrent > 0.0f) ? (float)height / maxCurrent : 1.0f;

    for (int i = 0; i < numPoints - 1; i++) {
        int x1 = x + (int)round((dutyCyclesVec[i] - MIN_DUTY_CYCLE_START) * xScale);
        int y1 = y + height - (int)round(currentsVec[i] * yScale);
        int x2 = x + (int)round((dutyCyclesVec[i + 1] - MIN_DUTY_CYCLE_START) * xScale);
        int y2 = y + height - (int)round(currentsVec[i + 1] * yScale);
        tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
    }

    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(1);
    tft.drawString("Duty Cycle (%)", x + width / 2 - 40, y + height + 20);
    tft.drawString("(A)", x - 30, y + height / 2);

    int tickStep = max(1, (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START) / 5);
    for (int i = MIN_DUTY_CYCLE_START; i <= MAX_DUTY_CYCLE; i += tickStep) {
        int xTick = x + (int)round((i - MIN_DUTY_CYCLE_START) * xScale);
        tft.drawLine(xTick, y + height, xTick, y + height + 5, TFT_WHITE);
        tft.drawNumber(i, xTick - 10, y + height + 5);
    }

    if (maxCurrent > 0.0f) {
        for (int k = 0; k <= 3; ++k) {
            float val = maxCurrent * (k / 3.0f);
            int yTick = y + height - (int)round(val * yScale);
            tft.drawLine(x - 5, yTick, x, yTick, TFT_WHITE);
            tft.drawFloat(val, 2, x - 30, yTick - 5);
        }
    }
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
}

// average of valid resistances from pairs
float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    float sum = 0.0f;
    int count = 0;
    for (float r : resistances) {
        if (r > MIN_VALID_RESISTANCE) {
            sum += r;
            count++;
        }
    }
    if (count > 0) {
        float average = sum / count;
        Serial.printf("\nAverage Internal Resistance (Pairs): %.3f Ohm (%d valid points)\n", average, count);
        return average;
    } else {
        Serial.println("\nNo valid internal resistance measurements from pairs.");
        return -1.0f;
    }
}

// regression helper for vectors (keeps your existing function available too)
void performLinearRegression(const std::vector<float>& voltages, const std::vector<float>& currents) {
    if (voltages.size() >= 2 && voltages.size() == currents.size()) {
        Serial.println("Calculating overall internal resistance using linear regression (Loaded Data)...");

        double sumI = std::accumulate(currents.begin(), currents.end(), 0.0);
        double sumV = std::accumulate(voltages.begin(), voltages.end(), 0.0);
        double sumII = std::inner_product(currents.begin(), currents.end(), currents.begin(), 0.0);
        double sumIV = 0.0;
        for (size_t i = 0; i < currents.size(); ++i) sumIV += currents[i] * voltages[i];

        double n = static_cast<double>(voltages.size());
        double denominator = (n * sumII - sumI * sumI);

        if (std::fabs(denominator) > 1e-9) {
            double overallInternalResistance = (n * sumIV - sumI * sumV) / denominator;
            double openCircuitVoltage = (sumV - overallInternalResistance * sumI) / n;
            Serial.printf("Calculated Overall Internal Resistance (Linear Regression on Loaded Data): %.3f Ohm\n", std::fabs(overallInternalResistance));
            Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", openCircuitVoltage);
        } else {
            Serial.println("Error: Could not calculate overall internal resistance (division by zero in regression).");
        }
    } else {
        Serial.println("Not enough/unequal data points to perform overall linear regression on loaded data.");
    }
}

// array helpers (kept almost identical but with guards)
int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count <= 0) return 0;
    int low = 0;
    int high = count - 1;
    int mid;
    while (low <= high) {
        mid = low + (high - low) / 2;
        if (data[mid][0] == targetCurrent) return mid;
        if (data[mid][0] < targetCurrent) low = mid + 1;
        else high = mid - 1;
    }
    if (low < count) {
        if (high >= 0 && (targetCurrent - data[high][0] < data[low][0] - targetCurrent)) return high;
        return low;
    } else {
        return high;
    }
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count < 0) count = 0;
    if (index < 0) index = 0;
    if (index > count) index = count;
    if (count >= MAX_RESISTANCE_POINTS) return;
    for (int i = count; i > index; --i) {
        data[i][0] = data[i - 1][0];
        data[i][1] = data[i - 1][1];
    }
    data[index][0] = current;
    data[index][1] = resistance;
    ++count;
}

void averageDataPoints(float data[][2], int index1, int index2) {
    if (index1 < 0 || index2 < 0) return;
    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index < 0 || index >= count) return;
    for (int i = index; i < count - 1; ++i) {
        data[i][0] = data[i + 1][0];
        data[i][1] = data[i + 1][1];
    }
    --count;
}

float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double mean = sum / data.size();
    double sqsum = 0.0;
    for (double v : data) sqsum += (v - mean) * (v - mean);
    return (float)std::sqrt(sqsum / data.size());
}

// store or blend data point with heuristics (kept similar, but robust)
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= 0.0f) return;

    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
        insertDataPoint(data, count, current, resistance, insertIndex);
    } else {
        int closestIndex = findClosestIndex(data, count, current);
        if (closestIndex < 0) closestIndex = 0;
        // compute isolation threshold based on spacing if possible
        float isolationThreshold = 0.02f;
        if (count >= 2) {
            std::vector<float> spacings;
            spacings.reserve(count - 1);
            for (int i = 1; i < count; ++i) spacings.push_back(data[i][0] - data[i - 1][0]);
            if (!spacings.empty()) {
                float meanSpacing = std::accumulate(spacings.begin(), spacings.end(), 0.0f) / spacings.size();
                float stdDevSpacing = standardDeviation(spacings);
                isolationThreshold = meanSpacing + 1.5f * stdDevSpacing;
                if (isolationThreshold <= 0.0f) isolationThreshold = 0.02f;
            }
        }

        bool isIsolated = true;
        if (count > 1) {
            float distanceToPrev = (closestIndex > 0) ? std::fabs(data[closestIndex][0] - data[closestIndex - 1][0]) : isolationThreshold * 2.0f;
            float distanceToNext = (closestIndex < count - 1) ? std::fabs(data[closestIndex][0] - data[closestIndex + 1][0]) : isolationThreshold * 2.0f;
            if (distanceToPrev < isolationThreshold || distanceToNext < isolationThreshold) isIsolated = false;
        } else {
            isIsolated = false;
        }

        if (isIsolated && count >= 2) {
            int index1 = (closestIndex > 0) ? closestIndex - 1 : -1;
            int index2 = (closestIndex < count - 1) ? closestIndex + 1 : -1;
            if (index1 != -1 && index2 != -1) {
                averageDataPoints(data, index1, index2);
                removeDataPoint(data, count, index2);
                int insertIndex = 0;
                while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
                insertDataPoint(data, count, current, resistance, insertIndex);
                return;
            } else {
                data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
                return;
            }
        } else {
            // simply average with the closest
            data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
            data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
            return;
        }
    }
}

void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return;
    for (int i = 0; i <= count - 4; ++i) {
        for (int j = i + 3; j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                std::vector<float> cluster_resistances;
                for (int k = i; k <= j; ++k) cluster_resistances.push_back(data[k][1]);
                if (cluster_resistances.size() >= 4) {
                    float avg = std::accumulate(cluster_resistances.begin(), cluster_resistances.end(), 0.0f) / cluster_resistances.size();
                    float stddev = standardDeviation(cluster_resistances);
                    std::vector<int> high_error_indices;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - avg) > error_threshold_multiplier * stddev) high_error_indices.push_back(k);
                    }
                    for (int idx : high_error_indices) data[idx][1] = avg;
                    i = j; // skip to next cluster
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
        Serial.println("Insufficient data points for linear regression.");
        return false;
    }
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < count; ++i) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += (double)data[i][0] * (double)data[i][1];
        sumX2 += (double)data[i][0] * (double)data[i][0];
    }
    double n = (double)count;
    double denom = n * sumX2 - sumX * sumX;
    if (std::fabs(denom) < 1e-9) {
        Serial.println("Denominator is too small for linear regression.");
        return false;
    }
    double s = (n * sumXY - sumX * sumY) / denom;
    double b = (sumY - s * sumX) / n;
    slope = (float)s;
    intercept = (float)b;
    return true;
}
