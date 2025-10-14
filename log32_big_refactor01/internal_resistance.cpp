#include "internal_resistance.h"
#include "definitions.h"

extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
extern void processThermistorData(const MeasurementData& data, const String& measurementType);

IRState currentIRState = IR_STATE_IDLE;
unsigned long irStateChangeTime = 0;
MeasurementData currentMeasurement;
IRState nextIRState;

void getSingleMeasurement(int dc, IRState nextState) {
    dutyCycle = dc;
    analogWrite(pwmPin, dutyCycle);
    irStateChangeTime = millis();
    nextIRState = nextState;
    currentIRState = IR_STATE_GET_MEASUREMENT;
}
int irDutyCycle = 0;
int minDutyCycle = 0;
int findMinDcLow = 0;
int findMinDcHigh = 0;
int findMinDcMid = 0;
int minimalDutyCycle = 0;
std::vector<std::pair<int, int>> dutyCyclePairs;
int pairIndex = 0;
float minCurrent = 0;
float maxCurrent = 0;
int lowDc = 0;
int previousHighDc = 0;
int pairGenerationStep = 0;
int pairGenerationSubStep = 0;
int lowBound = 0;
int highBound = 0;
int bestHighDc = 0;
float minCurrentDifference = 0;
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

void measureInternalResistanceStep() {
    unsigned long now = millis();

    switch (currentIRState) {
        case IR_STATE_IDLE:
            // Do nothing
            break;

        case IR_STATE_START:
            Serial.println("Starting improved internal resistance measurement...");
            resistanceDataCount = 0;
            resistanceDataCountPairs = 0;
            voltagesLoaded.clear();
            currentsLoaded.clear();
            dutyCycles.clear();
            consecutiveInternalResistances.clear();
            dutyCyclePairs.clear();
            pairIndex = 0;
            currentIRState = IR_STATE_STOP_LOAD_WAIT;
            irStateChangeTime = now;
            break;

        case IR_STATE_STOP_LOAD_WAIT:
            dutyCycle = 0;
            analogWrite(pwmPin, 0);
            if (now - irStateChangeTime >= UNLOADED_VOLTAGE_DELAY_MS) {
                currentIRState = IR_STATE_GET_UNLOADED_VOLTAGE;
            }
            break;

        case IR_STATE_GET_UNLOADED_VOLTAGE:
            {
                MeasurementData initialUnloaded;
                getThermistorReadings(initialUnloaded.temp1, initialUnloaded.temp2, initialUnloaded.tempDiff, initialUnloaded.t1_millivolts, initialUnloaded.voltage, initialUnloaded.current);
                Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);
                currentIRState = IR_STATE_FIND_MIN_DC;
            }
            break;

        case IR_STATE_FIND_MIN_DC:
            if (findMinDcLow == 0) {
                Serial.println("Finding minimal duty cycle for measurable current using binary search...");
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);
                tft.drawString("Finding Min Duty Cycle", 10, 10);
                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;
            }

            if (findMinDcLow == 0) {
                Serial.println("Finding minimal duty cycle for measurable current using binary search...");
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);
                tft.drawString("Finding Min Duty Cycle", 10, 10);
                findMinDcLow = MIN_DUTY_CYCLE_START;
                findMinDcHigh = MAX_DUTY_CYCLE;
                minimalDutyCycle = 0;
            }

            if (findMinDcLow <= findMinDcHigh) {
                findMinDcMid = findMinDcLow + (findMinDcHigh - findMinDcLow) / 2;
                getSingleMeasurement(findMinDcMid, IR_STATE_FIND_MIN_DC);
            } else {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE);
                tft.setTextSize(2);

                if (minimalDutyCycle > 0) {
                    Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
                    tft.printf("Minimal Duty Cycle Found:\n%d%%\n", minimalDutyCycle);
                    minDutyCycle = minimalDutyCycle;
                    currentIRState = IR_STATE_GENERATE_PAIRS;
                } else {
                    Serial.println("Warning: Could not find a duty cycle producing measurable current.");
                    tft.println("Warning: Could not find\nmeasurable current.");
                    currentIRState = IR_STATE_IDLE;
                }
            }

            if (currentMeasurement.dutyCycle == findMinDcMid) {
                if (currentMeasurement.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    minimalDutyCycle = findMinDcMid;
                    findMinDcHigh = findMinDcMid - 1;
                } else {
                    findMinDcLow = findMinDcMid + 1;
                }
            }
            break;

        case IR_STATE_GENERATE_PAIRS:
            if (pairGenerationStep == 0) {
                Serial.println("Generating duty cycle pairs...");
                if (minDutyCycle == 0) {
                    currentIRState = IR_STATE_IDLE;
                    break;
                }
                dutyCycle = minDutyCycle;
                analogWrite(pwmPin, dutyCycle);
                irStateChangeTime = now;
                pairGenerationStep = 1;
            } else if (pairGenerationStep == 1) {
                getSingleMeasurement(minDutyCycle, IR_STATE_GENERATE_PAIRS);
                pairGenerationStep = 2;
            } else if (pairGenerationStep == 2) {
                minCurrent = currentMeasurement.current;
                getSingleMeasurement(MAX_DUTY_CYCLE, IR_STATE_GENERATE_PAIRS);
                pairGenerationStep = 3;
            } else if (pairGenerationStep == 3) {
                maxCurrent = currentMeasurement.current;
                lowDc = minDutyCycle;
                previousHighDc = MAX_DUTY_CYCLE;
                pairIndex = 0;
                pairGenerationStep = 4;
            } else if (pairGenerationStep == 4) {
                if (pairGenerationSubStep == 0) {
                    if (pairIndex < MAX_RESISTANCE_POINTS / 2) {
                        float targetHighCurrent = maxCurrent - (pairIndex * (maxCurrent - minCurrent) / (MAX_RESISTANCE_POINTS / 2));
                        bestHighDc = -1;
                        minCurrentDifference = 1e9;
                        lowBound = minDutyCycle;
                        highBound = previousHighDc;
                        pairGenerationSubStep = 1;
                    } else {
                        currentIRState = IR_STATE_MEASURE_L_UL;
                        pairIndex = 0;
                    }
                } else if (pairGenerationSubStep == 1) {
                    if (lowBound <= highBound) {
                        int midDc = lowBound + (highBound - lowBound) / 2;
                        getSingleMeasurement(midDc, IR_STATE_GENERATE_PAIRS);
                        pairGenerationSubStep = 2;
                    } else {
                        int currentHighDc = bestHighDc != -1 ? bestHighDc : previousHighDc;
                        if (currentHighDc < minDutyCycle) currentHighDc = minDutyCycle;
                        dutyCyclePairs.push_back({lowDc, currentHighDc});
                        previousHighDc = currentHighDc - 1;
                        pairIndex++;
                        pairGenerationSubStep = 0;
                    }
                } else if (pairGenerationSubStep == 2) {
                    float targetHighCurrent = maxCurrent - (pairIndex * (maxCurrent - minCurrent) / (MAX_RESISTANCE_POINTS / 2));
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
            break;

        case IR_STATE_MEASURE_L_UL:
            if (pairIndex < dutyCyclePairs.size()) {
                if (measureStep == 0) {
                    int dc = dutyCyclePairs[pairIndex].second;
                    getSingleMeasurement(dc, IR_STATE_MEASURE_L_UL);
                    measureStep = 1;
                } else if (measureStep == 1) {
                    voltagesLoaded.push_back(currentMeasurement.voltage);
                    currentsLoaded.push_back(currentMeasurement.current);
                    ir_dutyCycles.push_back(static_cast<float>(currentMeasurement.dutyCycle));
                    getSingleMeasurement(0, IR_STATE_MEASURE_L_UL);
                    measureStep = 2;
                } else if (measureStep == 2) {
                    if (currentsLoaded.back() > 0.01f) {
                        float internalResistance = (currentMeasurement.voltage - voltagesLoaded.back()) / currentsLoaded.back();
                        storeResistanceData(currentsLoaded.back(), std::fabs(internalResistance), internalResistanceData, resistanceDataCount);
                    } else {
                        storeResistanceData(currentsLoaded.back(), -1.0f, internalResistanceData, resistanceDataCount);
                    }
                    pairIndex++;
                    measureStep = 0;
                }
            } else {
                currentIRState = IR_STATE_MEASURE_PAIRS;
                pairIndex = 0;
                measureStep = 0;
            }
            break;

        case IR_STATE_MEASURE_PAIRS:
            if (pairIndex < dutyCyclePairs.size()) {
                if (measureStep == 0) {
                    int dcLow = dutyCyclePairs[pairIndex].first;
                    getSingleMeasurement(dcLow, IR_STATE_MEASURE_PAIRS);
                    measureStep = 1;
                } else if (measureStep == 1) {
                    voltagesLoaded.push_back(currentMeasurement.voltage);
                    currentsLoaded.push_back(currentMeasurement.current);
                    int dcHigh = dutyCyclePairs[pairIndex].second;
                    getSingleMeasurement(dcHigh, IR_STATE_MEASURE_PAIRS);
                    measureStep = 2;
                } else if (measureStep == 2) {
                    if (currentMeasurement.current > currentsLoaded.back() + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistanceConsecutive = (voltagesLoaded.back() - currentMeasurement.voltage) / (currentMeasurement.current - currentsLoaded.back());
                        consecutiveInternalResistances.push_back(std::fabs(internalResistanceConsecutive));
                        storeResistanceData(currentMeasurement.current, std::fabs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
                    } else {
                        consecutiveInternalResistances.push_back(-1.0f);
                        storeResistanceData(currentMeasurement.current, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                    pairIndex++;
                    measureStep = 0;
                }
            } else {
                currentIRState = IR_STATE_COMPLETE;
            }
            break;

        case IR_STATE_GET_MEASUREMENT:
            if (now - irStateChangeTime >= STABILIZATION_DELAY_MS) {
                getThermistorReadings(currentMeasurement.temp1, currentMeasurement.temp2, currentMeasurement.tempDiff, currentMeasurement.t1_millivolts, currentMeasurement.voltage, currentMeasurement.current);
                currentMeasurement.dutyCycle = dutyCycle;
                currentMeasurement.timestamp = millis();
                currentIRState = nextIRState;
            }
            break;

        case IR_STATE_COMPLETE:
            bubbleSort(internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

            if (resistanceDataCount >= 2) {
                if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
            }

            if (resistanceDataCountPairs >= 2) {
                if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of paired resistance.");
            }

            Serial.printf("Internal resistance measurement complete. %d loaded/unloaded points, %d pair points collected.\n", resistanceDataCount, resistanceDataCountPairs);
            isMeasuringResistance = false;
            currentIRState = IR_STATE_IDLE;
            break;
    }
}

void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                float temp0 = data[j][0];
                float temp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0;
                data[j + 1][1] = temp1;
            }
        }
    }
}

void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count < MAX_RESISTANCE_POINTS && resistance > MIN_VALID_RESISTANCE) {
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
  uint16_t GREY = TFT_DARKGREY;
  const int DUTY_CYCLE_BAR_Y = 10;
  const int DUTY_CYCLE_BAR_HEIGHT = 10;
  const int DUTY_CYCLE_BAR_START_X = 30;
  const int DUTY_CYCLE_BAR_END_X = SCREEN_WIDTH - 2;

  tft.fillRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, GREY);
  tft.drawRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, TFT_WHITE);

  int range = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START;
  if (range == 0) range = 1;

  int low_x = map(low, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
  int high_x = map(high, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
  int mid_x = map(mid, MIN_DUTY_CYCLE_START, MAX_DUTY_CYCLE, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);

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

void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, int x, int y, int width, int height) {
  if (numPoints <= 1) return;

  tft.fillRect(x, y, width, height, TFT_BLACK);
  tft.drawRect(x, y, width, height, TFT_WHITE);

  float xScale = (float)width / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START);
  float yScale = (height > 0 && maxCurrent > 0) ? (float)height / maxCurrent : 0;

  for (int i = 0; i < numPoints - 1; i++) {
    int x1 = x + (dutyCycles[i] - MIN_DUTY_CYCLE_START) * xScale;
    int y1 = y + height - currents[i] * yScale;
    int x2 = x + (dutyCycles[i + 1] - MIN_DUTY_CYCLE_START) * xScale;
    int y2 = y + height - currents[i + 1] * yScale;
    tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
  }

  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.drawString("Duty Cycle (%)", x + width / 2 - 40, y + height + 20);
  tft.drawString("(A)", x - 30, y + height / 2);

  for (int i = MIN_DUTY_CYCLE_START; i <= MAX_DUTY_CYCLE; i += (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE_START) / 5) {
    int xTick = x + (i - MIN_DUTY_CYCLE_START) * xScale;
    tft.drawLine(xTick, y + height, xTick, y + height + 5, TFT_WHITE);
    tft.drawNumber(i, xTick - 10, y + height + 5);
  }

  if (maxCurrent > 0) {
    for (float i = 0; i <= maxCurrent; i += maxCurrent / 3) {
      int yTick = y + height - i * yScale;
      tft.drawLine(x - 5, yTick, x, yTick, TFT_WHITE);
      tft.drawFloat(i, 2, x - 30, yTick - 5);
    }
  }
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
}

float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    float sum = 0;
    int count = 0;
    for (float rint : resistances) {
        if (rint > MIN_VALID_RESISTANCE) {
            sum += rint;
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

void performLinearRegression(const std::vector<float>& voltages, const std::vector<float>& currents) {
    if (voltages.size() >= 2) {
        Serial.println("Calculating overall internal resistance using linear regression (Loaded Data)...");

        double sumI = std::accumulate(currents.begin(), currents.end(), 0.0);
        double sumV = std::accumulate(voltages.begin(), voltages.end(), 0.0);
        double sumII = std::inner_product(currents.begin(), currents.end(), currents.begin(), 0.0);
        double sumIV = 0.0;
        for (size_t i = 0; i < currents.size(); ++i) {
            sumIV += currents[i] * voltages[i];
        }

        int n = voltages.size();
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
        Serial.println("Not enough data points to perform overall linear regression on loaded data.");
    }
}


int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    int low = 0;
    int high = count - 1;
    int mid;

    while (low <= high) {
        mid = low + (high - low) / 2;
        if (data[mid][0] == targetCurrent) {
            return mid;
        } else if (data[mid][0] < targetCurrent) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    if (low < count) {
        if (high >= 0 && (targetCurrent - data[high][0] < data[low][0] - targetCurrent)) {
            return high;
        } else {
            return low;
        }
    } else {
        return high;
    }
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count < MAX_RESISTANCE_POINTS) {
        for (int i = count; i > index; --i) {
            data[i][0] = data[i - 1][0];
            data[i][1] = data[i - 1][1];
        }
        data[index][0] = current;
        data[index][1] = resistance;
        count++;
    }
}

void averageDataPoints(float data[][2], int index1, int index2) {
    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index >= 0 && index < count) {
        for (int i = index; i < count - 1; ++i) {
            data[i][0] = data[i + 1][0];
            data[i][1] = data[i + 1][1];
        }
        count--;
    }
}

float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;
    float sum = std::accumulate(data.begin(), data.end(), 0.0f);
    float mean = sum / data.size();
    std::vector<float> diffSq(data.size());
    std::transform(data.begin(), data.end(), diffSq.begin(),
                   [mean](float x){ return std::pow(x - mean, 2); });
    float sqSum = std::accumulate(diffSq.begin(), diffSq.end(), 0.0f);
    return std::sqrt(sqSum / data.size());
}

void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= 0) return;

    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) {
            insertIndex++;
        }
        insertDataPoint(data, count, current, resistance, insertIndex);
    } else {
        int closestIndex = findClosestIndex(data, count, current);
        if (closestIndex >= 0 && closestIndex < MAX_RESISTANCE_POINTS) {

            float isolationThreshold = 0.0f;

            if (count >= 2) {
                std::vector<float> spacings;
                for (int i = 1; i < count; ++i) {
                    spacings.push_back(data[i][0] - data[i - 1][0]);
                }

                if (!spacings.empty()) {
                    float sum = std::accumulate(spacings.begin(), spacings.end(), 0.0f);
                    float meanSpacing = sum / spacings.size();
                    float stdDevSpacing = standardDeviation(spacings);

                    isolationThreshold = meanSpacing + 1.5f * stdDevSpacing;
                    if (isolationThreshold <= 0) isolationThreshold = 0.02f;
                } else {
                    isolationThreshold = 0.02f;
                }
            } else {
                isolationThreshold = 0.02f;
            }

            bool isIsolated = true;
            if (count > 1) {
                float distanceToPrev = (closestIndex > 0) ? std::fabs(data[closestIndex][0] - data[closestIndex - 1][0]) : isolationThreshold * 2.0f;
                float distanceToNext = (closestIndex < count - 1) ? std::fabs(data[closestIndex][0] - data[closestIndex + 1][0]) : isolationThreshold * 2.0f;

                if (distanceToPrev < isolationThreshold || distanceToNext < isolationThreshold) {
                    isIsolated = false;
                }
            } else {
                isIsolated = false;
            }

            if (isIsolated && count >= 2) {
                int index1 = -1, index2 = -1;

                if (closestIndex > 0) {
                    index1 = closestIndex - 1;
                }
                if (closestIndex < count - 1) {
                    index2 = closestIndex + 1;
                }

                if (index1 != -1 && index2 != -1) {
                    averageDataPoints(data, index1, index2);
                    removeDataPoint(data, count, index2);
                    int insertIndex = 0;
                    while (insertIndex < count && data[insertIndex][0] < current) {
                        insertIndex++;
                    }
                    insertDataPoint(data, count, current, resistance, insertIndex);
                } else {
                    data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                    data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
                }
            } else {
                data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
            }
        }
    }
}

void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return;

    for (int i = 0; i <= count - 4; ++i) {
        for (int j = i + 3; j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                std::vector<float> cluster_resistances;
                for (int k = i; k <= j; ++k) {
                    cluster_resistances.push_back(data[k][1]);
                }

                if (cluster_resistances.size() >= 4) {
                    float sum_resistance = std::accumulate(cluster_resistances.begin(), cluster_resistances.end(), 0.0f);
                    float average_resistance = sum_resistance / cluster_resistances.size();
                    float std_dev_resistance = standardDeviation(cluster_resistances);

                    std::vector<int> high_error_indices;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - average_resistance) > error_threshold_multiplier * std_dev_resistance) {
                            high_error_indices.push_back(k);
                        }
                    }

                    if (!high_error_indices.empty()) {
                        for (int index : high_error_indices) {
                            data[index][1] = average_resistance;
                        }
                    }
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
        Serial.println("Insufficient data points for linear regression.");
        return false;
    }

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumXY = 0.0f;
    float sumX2 = 0.0f;

    for (int i = 0; i < count; ++i) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += data[i][0] * data[i][1];
        sumX2 += data[i][0] * data[i][0];
    }

    float n = static_cast<float>(count);
    float denominator = n * sumX2 - sumX * sumX;

    if (std::fabs(denominator) < 1e-6) {
        Serial.println("Denominator is too small for linear regression.");
        return false;
    }

    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / n;

    return true;
}
