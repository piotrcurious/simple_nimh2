#include "internal_resistance.h"
#include "definitions.h"

float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;
bool isMeasuringResistance = false;

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

void stopLoad() {
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    delay(UNLOADED_VOLTAGE_DELAY_MS);
}

MeasurementData getUnloadedVoltageMeasurement() {
    stopLoad();
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = 0;
    data.timestamp = millis();
    return data;
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

int findMinimalDutyCycle() {
  Serial.println("Finding minimal duty cycle for measurable current using binary search...");
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawString("Finding Min Duty Cycle", 10, 10);

  int low = MIN_DUTY_CYCLE_START;
  int high = MAX_DUTY_CYCLE;
  int minimalDutyCycle = 0;
  int iteration = 0;

  const int GRAPH_X = 30;
  const int GRAPH_Y = 50;
  const int GRAPH_WIDTH = SCREEN_WIDTH - 2;
  const int GRAPH_HEIGHT = SCREEN_HEIGHT-90;
  const int MAX_GRAPH_POINTS = 100;
  int dutyCyclePoints[MAX_GRAPH_POINTS];
  float currentPoints[MAX_GRAPH_POINTS];
  int numPoints = 0;
  float maxCurrent = 0.0;

  while (low <= high) {
    iteration++;
    int mid = low + (high - low) / 2;

    Serial.printf("Iteration: %d, Testing duty cycle: %d\n", iteration, mid);
    MeasurementData data = takeMeasurement(mid, STABILIZATION_DELAY_MS);
    Serial.printf("Measured current at %d%% duty cycle: %.3f A\n", mid, data.current);
    stopLoad();

    tft.fillRect(0, 10, SCREEN_WIDTH-30, SCREEN_HEIGHT - 10, TFT_BLACK);
    drawDutyCycleBar(low, high, mid, data.current, MEASURABLE_CURRENT_THRESHOLD);

    if (numPoints < MAX_GRAPH_POINTS) {
      dutyCyclePoints[numPoints] = mid;
      currentPoints[numPoints] = data.current;
      numPoints++;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      }
    } else {
      for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
        dutyCyclePoints[i] = dutyCyclePoints[i + 1];
        currentPoints[i] = currentPoints[i + 1];
      }
      dutyCyclePoints[MAX_GRAPH_POINTS - 1] = mid;
      currentPoints[MAX_GRAPH_POINTS - 1] = data.current;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      } else {
        maxCurrent = 0.0;
        for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
          if (currentPoints[i] > maxCurrent) {
            maxCurrent = currentPoints[i];
          }
        }
      }
    }
    drawGraph(dutyCyclePoints, currentPoints, numPoints, maxCurrent, GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT);

    if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
      minimalDutyCycle = mid;
      high = mid - 1;
      Serial.printf("Found measurable current at %d%%, trying lower values.\n", mid);
    } else {
      low = mid + 1;
      Serial.printf("Current too low at %d%%, trying higher values.\n", mid);
    }
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);

  if (minimalDutyCycle > 0) {
    Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
    tft.printf("Minimal Duty Cycle Found:\n%d%%\n", minimalDutyCycle);
    return minimalDutyCycle;
  } else {
    Serial.println("Warning: Could not find a duty cycle producing measurable current.");
    tft.println("Warning: Could not find\nmeasurable current.");
    return 0;
  }
}

std::vector<std::pair<int, int>> generateDutyCyclePairs(int minDutyCycle) {
    Serial.println("Generating duty cycle pairs (improved for linear current spacing using binary search)...");
    std::vector<std::pair<int, int>> pairs;
    if (minDutyCycle == 0) {
        return pairs;
    }

    int numPairs = MAX_RESISTANCE_POINTS / 2;
    if (numPairs < 1) {
        Serial.println("MAX_RESISTANCE_POINTS should be at least 2 for paired measurement.");
        return pairs;
    }

    MeasurementData minCurrentData = takeMeasurement(minDutyCycle, STABILIZATION_DELAY_MS);
    processThermistorData(minCurrentData, "Estimating Min Current");
    float minCurrent = minCurrentData.current;

    MeasurementData maxCurrentData = takeMeasurement(MAX_DUTY_CYCLE, STABILIZATION_DELAY_MS);
    processThermistorData(maxCurrentData, "Estimating Max Current");
    float maxCurrent = maxCurrentData.current;

    if (maxCurrent <= minCurrent) {
        Serial.println("Warning: Maximum current is not greater than minimum current. Cannot ensure linear spacing. Falling back to a simpler approach.");
        int highDc = MAX_DUTY_CYCLE;
        int lowDc = minDutyCycle;
        int dutyCycleStep = (MAX_DUTY_CYCLE - minDutyCycle) / numPairs;
        for (int i = 0; i < numPairs; ++i) {
            if (highDc < lowDc) break;
            MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
            processThermistorData(lowData, "Generating Pairs (Fallback)");
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < highDc) {
                int adjustedLowDc = lowDc;
                for (int j = 0; j < 10; ++j) {
                    adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                    if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                    MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                    if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                        lowDc = adjustedLowDc;
                        Serial.printf("Adjusting low duty cycle to %d due to low current (fallback).\n", lowDc);
                        break;
                    }
                }
                if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                    Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current (fallback).");
                }
            } else if (lowDc > MAX_DUTY_CYCLE) {
                Serial.println("Warning: Low duty cycle exceeded maximum value (fallback).");
                break;
            }
            pairs.push_back({lowDc, highDc});
            highDc -= dutyCycleStep;
            lowDc += dutyCycleStep;
            if (highDc < minDutyCycle) highDc = minDutyCycle + 1;
            if (lowDc > MAX_DUTY_CYCLE) lowDc = MAX_DUTY_CYCLE - 1;
        }
        Serial.printf("Generated %zu duty cycle pairs (fallback).\n", pairs.size());
        return pairs;
    }

    float totalCurrentRange = maxCurrent - minCurrent;
    float desiredCurrentIncrement = totalCurrentRange / numPairs;

    int lowDc = minDutyCycle;
    int previousHighDc = MAX_DUTY_CYCLE;

    for (int i = 0; i < numPairs; ++i) {
        float targetHighCurrent = maxCurrent - (i * desiredCurrentIncrement);
        int bestHighDc = -1;
        float minCurrentDifference = 1e9;

        int lowBound = minDutyCycle;
        int highBound = previousHighDc;

        while (lowBound <= highBound) {
            int midDc = lowBound + (highBound - lowBound) / 2;
            MeasurementData midData = takeMeasurement(midDc, STABILIZATION_PAIRS_FIND_DELAY_MS);
            processThermistorData(midData, "Binary Searching High DC");
            float currentDifference = std::fabs(midData.current - targetHighCurrent);

            if (currentDifference < minCurrentDifference) {
                minCurrentDifference = currentDifference;
                bestHighDc = midDc;
            }

            if (midData.current > targetHighCurrent) {
                highBound = midDc - 1;
            } else {
                lowBound = midDc + 1;
            }
        }

        int currentHighDc = bestHighDc != -1 ? bestHighDc : previousHighDc;
        if (currentHighDc < minDutyCycle) currentHighDc = minDutyCycle;

        MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
        processThermistorData(lowData, "Generating Pairs");
             tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
             tft.printf("progress: %.0f ", ((float)i / numPairs) * 100.0);

        if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < currentHighDc) {
            int adjustedLowDc = lowDc;
            for (int j = 0; j < 5; ++j) {
                adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    lowDc = adjustedLowDc;
                    Serial.printf("Adjusting low duty cycle to %d due to low current.\n", lowDc);
                    break;
                }
            }
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current.");
            }
        } else if (lowDc > MAX_DUTY_CYCLE) {
            Serial.println("Warning: Low duty cycle exceeded maximum value.");
            break;
        }

        if (currentHighDc < lowDc) {
            currentHighDc = lowDc + 1;
            break;
        };

        pairs.push_back({lowDc, currentHighDc});
        previousHighDc = currentHighDc - 1;

        if (previousHighDc < minDutyCycle) break;
    }

    Serial.printf("Generated %zu duty cycle pairs (for linear current spacing using binary search).\n", pairs.size());
    return pairs;
}

void measureInternalResistanceLoadedUnloaded(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& voltagesLoaded, std::vector<float>& currentsLoaded, std::vector<float>& dutyCycles) {
    Serial.println("\n--- Measuring Internal Resistance (Loaded/Unloaded) ---");
    for (const auto& pair : dutyCyclePairs) {
        int dc = pair.second;
        Serial.printf("--- Duty Cycle (Loaded/Unloaded): %d ---\n", dc);

        MeasurementData loadedData = takeMeasurement(dc,STABILIZATION_DELAY_MS);
        processThermistorData(loadedData, "Rint L/UL");

        voltagesLoaded.push_back(loadedData.voltage);
        currentsLoaded.push_back(loadedData.current);
        dutyCycles.push_back(static_cast<float>(dc));

        Serial.printf("Duty Cycle ON (%d): Voltage: %.3f V, Current: %.3f A\n", dc, loadedData.voltage, loadedData.current);

        MeasurementData unloadedData = getUnloadedVoltageMeasurement();
        Serial.printf("Duty Cycle OFF: Voltage: %.3f V, Current: %.3f A\n", unloadedData.voltage, unloadedData.current);

        if (loadedData.current > 0.01f) {
            float internalResistance = (unloadedData.voltage - loadedData.voltage) / loadedData.current;
            storeResistanceData(loadedData.current, std::fabs(internalResistance), internalResistanceData, resistanceDataCount);
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded): %.3f Ohm\n", std::fabs(internalResistance));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(L/UL): %.3f ", std::fabs(internalResistance));
        } else {
            Serial.println("Warning: Current is too low to reliably calculate internal resistance (Loaded-Unloaded).");
            storeResistanceData(loadedData.current, -1.0f, internalResistanceData, resistanceDataCount);
        }
        Serial.println("---------------------------\n");
    }
}

void measureInternalResistancePairs(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& consecutiveInternalResistances) {
    Serial.println("\n--- Measuring Internal Resistance using Duty Cycle Pairs ---");
    for (const auto& pair : dutyCyclePairs) {
        int dcLow = pair.first;
        int dcHigh = pair.second;

        Serial.printf("--- Duty Cycle Pair: Low=%d, High=%d ---\n", dcLow, dcHigh);

        MeasurementData lowData = takeMeasurement(dcLow,STABILIZATION_DELAY_MS);
        Serial.printf("Duty Cycle Low (%d): Voltage: %.3f V, Current: %.3f A\n", dcLow, lowData.voltage, lowData.current);

        MeasurementData highData = takeMeasurement(dcHigh,STABILIZATION_DELAY_MS);
        processThermistorData(highData, "Rint Pair");
        Serial.printf("Duty Cycle High (%d): Voltage: %.3f V, Current: %.3f A\n", dcHigh, highData.voltage, highData.current);

        if (highData.current > lowData.current + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
            float internalResistanceConsecutive = (lowData.voltage - highData.voltage) / (highData.current - lowData.current);
            consecutiveInternalResistances.push_back(std::fabs(internalResistanceConsecutive));
            storeResistanceData(highData.current, std::fabs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
            Serial.printf("Calculated Internal Resistance (Pair): %.3f Ohm\n", std::fabs(internalResistanceConsecutive));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 50);
            tft.printf("(Pair): %.3f ", std::fabs(internalResistanceConsecutive));
        } else {
            Serial.println("Warning: Current difference is too small to reliably calculate internal resistance (Pair).");
            consecutiveInternalResistances.push_back(-1.0f);
            storeResistanceData(highData.current, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
        }
        Serial.println("---------------------------\n");
    }
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

void measureInternalResistance() {
    if (!isMeasuringResistance) return;

    Serial.println("Starting improved internal resistance measurement...");

    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;

    std::vector<float> voltagesLoaded;
    std::vector<float> currentsLoaded;
    std::vector<float> dutyCycles;
    std::vector<float> consecutiveInternalResistances;
    std::vector<float> unloadedVoltagesHistory;
    std::vector<unsigned long> unloadedVoltageTimestamps;

    MeasurementData initialUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(initialUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(initialUnloaded.timestamp);
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

    int minDutyCycle = findMinimalDutyCycle();
    if (minDutyCycle == 0) {
        return;
    }

    std::vector<std::pair<int, int>> dutyCyclePairs = generateDutyCyclePairs(minDutyCycle);
    measureInternalResistanceLoadedUnloaded(dutyCyclePairs, voltagesLoaded, currentsLoaded, dutyCycles);
    measureInternalResistancePairs(dutyCyclePairs, consecutiveInternalResistances);

    MeasurementData finalUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(finalUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(finalUnloaded.timestamp);
    Serial.printf("Final Unloaded Voltage: %.3f V\n", finalUnloaded.voltage);

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