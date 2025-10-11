#include "graphing.h"
#include "logging.h"
#include <TFT_eSPI.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>
#include <ctime>
#include <cstdlib>
#include <ArduinoEigenDense.h>

// --- Extern variables ---
extern TFT_eSPI tft;
extern float temp1_values[];
extern float temp2_values[];
extern float diff_values[];
extern float voltage_values[];
extern float current_values[];
extern const int PLOT_WIDTH;
extern const int PLOT_HEIGHT;
extern const int PLOT_X_START;
extern const int PLOT_Y_START;
extern const float MIN_TEMP;
extern const float MAX_TEMP;
extern const float MIN_DIFF_TEMP;
extern float MAX_DIFF_TEMP;
extern const float MIN_VOLTAGE;
extern const float MAX_VOLTAGE;
extern const float MIN_CURRENT;
extern const float MAX_CURRENT;
extern const uint16_t PLOT_X_AXIS_COLOR;
extern const uint16_t PLOT_Y_AXIS_COLOR;
extern const uint16_t PLOT_ZERO_COLOR;
extern const uint16_t GRAPH_COLOR_1;
extern const uint16_t GRAPH_COLOR_2;
extern const uint16_t GRAPH_COLOR_DIFF;
extern const uint16_t GRAPH_COLOR_VOLTAGE;
extern const uint16_t GRAPH_COLOR_CURRENT;
extern const uint16_t GRAPH_COLOR_RESISTANCE;
extern const uint16_t GRAPH_COLOR_RESISTANCE_PAIR;
extern const int LABEL_Y_START;
extern const int LABEL_TEXT_SIZE;
extern float internalResistanceData[][2];
extern int resistanceDataCount;
extern float internalResistanceDataPairs[][2];
extern int resistanceDataCountPairs;
extern float regressedInternalResistanceSlope;
extern float regressedInternalResistanceIntercept;
extern float regressedInternalResistancePairsSlope;
extern float regressedInternalResistancePairsIntercept;
extern std::vector<ChargeLogData> chargeLog;
extern float estimateTempDiff(float, float, float, float, float, uint32_t, uint32_t, float, float, float, float, float, float);
extern const float MAX_TEMP_DIFF_THRESHOLD;
extern int dutyCycle;

// --- Function Implementations ---

float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void bigUglyMessage(const String& measurementType) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
    tft.print(measurementType);
}

void drawDutyCycleBar(int low, int high, int mid, float current, float threshold) {
    const int DUTY_CYCLE_BAR_Y = 10;
    const int DUTY_CYCLE_BAR_HEIGHT = 10;
    const int DUTY_CYCLE_BAR_START_X = 30;
    const int DUTY_CYCLE_BAR_END_X = 320 - 2;

    tft.fillRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, TFT_DARKGREY);
    tft.drawRect(DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_Y, DUTY_CYCLE_BAR_END_X - DUTY_CYCLE_BAR_START_X + 1, DUTY_CYCLE_BAR_HEIGHT, TFT_WHITE);

    int range = 255 - 8;
    if (range == 0) range = 1;

    int low_x = map(low, 8, 255, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
    int high_x = map(high, 8, 255, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);
    int mid_x = map(mid, 8, 255, DUTY_CYCLE_BAR_START_X, DUTY_CYCLE_BAR_END_X);

    tft.drawLine(low_x, DUTY_CYCLE_BAR_Y - 10, low_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, TFT_WHITE);
    tft.drawLine(high_x, DUTY_CYCLE_BAR_Y - 10, high_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT + 10, TFT_WHITE);

    uint16_t mid_color = (current >= 0.005f) ? TFT_GREEN : TFT_RED;
    tft.fillCircle(mid_x, DUTY_CYCLE_BAR_Y + DUTY_CYCLE_BAR_HEIGHT / 2, 8, mid_color);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.printf("Low: %d%%, High: %d%%, Mid: %d%%\n", low, high, mid);
    tft.printf("Current: %.3f A (Threshold: %.3f A)\n", current, 0.005f);
    tft.printf("Measurable: %s\n", (current >= 0.005f) ? "Yes" : "No");
}

void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, int x, int y, int width, int height) {
    if (numPoints <= 1) return;

    tft.fillRect(x, y, width, height, TFT_BLACK);
    tft.drawRect(x, y, width, height, TFT_WHITE);

    float xScale = (float)width / (255 - 8);
    float yScale = (height > 0 && maxCurrent > 0) ? (float)height / maxCurrent : 0;

    for (int i = 0; i < numPoints - 1; i++) {
        int x1 = x + (dutyCycles[i] - 8) * xScale;
        int y1 = y + height - currents[i] * yScale;
        int x2 = x + (dutyCycles[i + 1] - 8) * xScale;
        int y2 = y + height - currents[i + 1] * yScale;
        tft.drawLine(x1, y1, x2, y2, TFT_GREEN);
    }

    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(1);
    tft.drawString("Duty Cycle (%)", x + width / 2 - 40, y + height + 20);
    tft.drawString("(A)", x - 30, y + height / 2);

    for (int i = 8; i <= 255; i += (255 - 8) / 5) {
        int xTick = x + (i - 8) * xScale;
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

namespace { // Anonymous namespace to limit scope of helper functions

struct DataPoint {
    float current;
    float resistance;
};

std::vector<DataPoint> extractValidData(float (*rawData)[2], int dataCount) {
    std::vector<DataPoint> validData;
    if (rawData != nullptr) {
        for (int i = 0; i < dataCount; ++i) {
            if (rawData[i][1] != -1.0f) {
                validData.push_back({rawData[i][0], rawData[i][1]});
            }
        }
    }
    return validData;
}

void findMinMax(const std::vector<DataPoint>& dataPoints, float& minCurrent, float& maxCurrent, float& minResistance, float& maxResistance) {
    for (const auto& dataPoint : dataPoints) {
        minCurrent = std::min(minCurrent, dataPoint.current);
        maxCurrent = std::max(maxCurrent, dataPoint.current);
        minResistance = std::min(minResistance, dataPoint.resistance);
        maxResistance = std::max(maxResistance, dataPoint.resistance);
    }
}

void plotResistanceData(const std::vector<DataPoint>& dataPoints, uint16_t color, float minCurrent, float maxCurrent, float minResistance, float maxResistance, int graphXStart, int graphYStart, int graphXEnd, int graphYEnd) {
    tft.setTextColor(color);
    if (dataPoints.size() >= 2) {
        for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
            float current1 = dataPoints[i].current;
            float resistance1 = dataPoints[i].resistance;
            float current2 = dataPoints[i + 1].current;
            float resistance2 = dataPoints[i + 1].resistance;

            int x1 = mapf(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = mapf(resistance1, minResistance, maxResistance, graphYEnd, graphYStart);

            int x2 = mapf(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = mapf(resistance2, minResistance, maxResistance, graphYEnd, graphYStart);

            tft.drawLine(x1, y1, x2, y2, color);
        }
    }
}

} // namespace

void displayInternalResistanceGraph() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    std::vector<DataPoint> validResistanceData = extractValidData(internalResistanceData, resistanceDataCount);
    std::vector<DataPoint> validInternalResistancePairs = extractValidData(internalResistanceDataPairs, resistanceDataCountPairs);

    if (validResistanceData.size() < 2 && validInternalResistancePairs.size() < 2) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("Not enough valid data");
        return;
    }

    float minCurrent = std::numeric_limits<float>::max();
    float maxCurrent = std::numeric_limits<float>::min();
    float minResistance = std::numeric_limits<float>::max();
    float maxResistance = std::numeric_limits<float>::min();

    bool dataFound = false;

    if (!validResistanceData.empty()) {
        findMinMax(validResistanceData, minCurrent, maxCurrent, minResistance, maxResistance);
        dataFound = true;
    }
    if (!validInternalResistancePairs.empty()) {
        findMinMax(validInternalResistancePairs, minCurrent, maxCurrent, minResistance, maxResistance);
        dataFound = true;
    }

    if (!dataFound) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("No valid data to plot");
        return;
    }

    auto adjustRange =[](float& minVal, float& maxVal, float paddingFactor) {
        float range = maxVal - minVal;
        if (range > 0) {
            minVal -= range * paddingFactor;
            maxVal += range * paddingFactor;
        } else if (maxVal == minVal) {
            minVal -= 0.1f;
            maxVal += 0.1f;
        }
    };

    adjustRange(minCurrent, maxCurrent, 0.1f);
    adjustRange(minResistance, maxResistance, 0.01f);

    if (minCurrent > maxCurrent) std::swap(minCurrent, maxCurrent);
    if (minResistance > maxResistance) std::swap(minResistance, maxResistance);

    int graphXStart = PLOT_X_START + 40;
    int graphYStart = PLOT_Y_START + 20;
    int graphXEnd = PLOT_X_START + PLOT_WIDTH - 20;
    int graphYEnd = PLOT_Y_START + PLOT_HEIGHT - 40;
    int graphWidth = graphXEnd - graphXStart;
    int graphHeight = graphYEnd - graphYStart;

    tft.drawLine(graphXStart, graphYEnd, graphXEnd, graphYEnd, PLOT_X_AXIS_COLOR);
    tft.drawLine(graphXStart, graphYEnd, graphXStart, graphYStart, PLOT_Y_AXIS_COLOR);

    plotResistanceData(validResistanceData, GRAPH_COLOR_RESISTANCE, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd);
    plotResistanceData(validInternalResistancePairs, GRAPH_COLOR_RESISTANCE_PAIR, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(graphXStart + graphWidth / 2 - 30, graphYEnd + 10);
    tft.println("Current (A)");

    tft.setCursor(PLOT_X_START + 5, graphYStart + graphHeight / 2 - 5);
    tft.print("R");

    tft.setCursor(graphXEnd - 50, graphYEnd + 10);
    tft.printf("%.2fA", maxCurrent);
    tft.setCursor(graphXStart + 5, graphYEnd + 10);
    tft.printf("%.2fA", minCurrent);

    tft.setCursor(graphXStart - 30, graphYStart - 10);
    tft.printf("%.2f", maxResistance);
    tft.setCursor(graphXStart - 30, graphYEnd - 10);
    tft.printf("%.2f", minResistance);

    if (dataFound) {
        if (resistanceDataCount >= 2) {
            float resistanceAtMinCurrent = regressedInternalResistanceSlope * minCurrent + regressedInternalResistanceIntercept;
            float resistanceAtMaxCurrent = regressedInternalResistanceSlope * maxCurrent + regressedInternalResistanceIntercept;

            if ((resistanceAtMinCurrent >= minResistance && resistanceAtMinCurrent <= maxResistance) ||
                (resistanceAtMaxCurrent >= minResistance && resistanceAtMaxCurrent <= maxResistance)) {
                int y1 = mapf(resistanceAtMinCurrent, minResistance, maxResistance, graphYEnd, graphYStart);
                int y2 = mapf(resistanceAtMaxCurrent, minResistance, maxResistance, graphYEnd, graphYStart);

                tft.drawLine(graphXStart, y1, graphXEnd, y2, TFT_WHITE);

                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE);
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "Rint(LU): %.2f + %.2f*I", regressedInternalResistanceIntercept, regressedInternalResistanceSlope);
                tft.setCursor(graphXEnd - 150, y2 - 10);
                tft.print(buffer);
            } else {
                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE);
                tft.setCursor(graphXStart + 10, graphYStart + 10);
                tft.print("Rint(LU) out of range");
            }
        } else {
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(graphXStart + 10, graphYStart + 20);
            tft.print("No Rint(LU) regression");
        }
    }

    if (dataFound) {
        if (resistanceDataCountPairs >= 2) {
            float resistanceAtMinCurrentPairs = regressedInternalResistancePairsSlope * minCurrent + regressedInternalResistancePairsIntercept;
            float resistanceAtMaxCurrentPairs = regressedInternalResistancePairsSlope * maxCurrent + regressedInternalResistancePairsIntercept;

            if ((resistanceAtMinCurrentPairs >= minResistance && resistanceAtMinCurrentPairs <= maxResistance) ||
                (resistanceAtMaxCurrentPairs >= minResistance && resistanceAtMaxCurrentPairs <= maxResistance)) {
                int y1 = mapf(resistanceAtMinCurrentPairs, minResistance, maxResistance, graphYEnd, graphYStart);
                int y2 = mapf(resistanceAtMaxCurrentPairs, minResistance, maxResistance, graphYEnd, graphYStart);

                tft.drawLine(graphXStart, y1, graphXEnd, y2, TFT_GREEN);

                tft.setTextSize(1);
                tft.setTextColor(TFT_GREEN);
                char buffer[50];
                snprintf(buffer, sizeof(buffer), "Rint(Pair): %.2f + %.2f*I", regressedInternalResistancePairsIntercept, regressedInternalResistancePairsSlope);
                tft.setCursor(graphXEnd - 150, y2 + 10);
                tft.print(buffer);
            } else {
                tft.setTextSize(1);
                tft.setTextColor(TFT_GREEN);
                tft.setCursor(graphXStart + 10, graphYStart + 30);
                tft.print("Rint(Pair) out of range");
            }
        } else {
            tft.setTextSize(1);
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(graphXStart + 10, graphYStart + 40);
            tft.print("No Rint(Pair) regression");
        }
    }

    tft.setTextSize(1);
    tft.setTextColor(GRAPH_COLOR_RESISTANCE);
    tft.setCursor(graphXStart + 10, PLOT_Y_START + 10);
    tft.print("R_int(1)");

    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
    tft.setCursor(graphXStart + 10, PLOT_Y_START + 20);
    tft.print("R_int(2)");
}

void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current) {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        temp1_values[i] = temp1_values[i + 1];
        temp2_values[i] = temp2_values[i + 1];
        diff_values[i] = diff_values[i + 1];
        voltage_values[i] = voltage_values[i + 1];
        current_values[i] = current_values[i + 1];
    }
    temp1_values[PLOT_WIDTH - 1] = temp1;
    temp2_values[PLOT_WIDTH - 1] = temp2;
    diff_values[PLOT_WIDTH - 1] = tempDiff;
    voltage_values[PLOT_WIDTH - 1] = voltage;
    current_values[PLOT_WIDTH - 1] = current;
}

uint16_t darkerColor(uint16_t color, float darkeningFactor) {
    darkeningFactor = max(0.0f, min(1.0f, darkeningFactor));
    float grayingFactor = 0.6f;

    uint8_t r = (color >> 11) & 0x1F;
    uint8_t g = (color >> 5) & 0x3F;
    uint8_t b = color & 0x1F;

    float fr = r / 31.0f;
    float fg = g / 63.0f;
    float fb = b / 31.0f;

    float luminance = 0.2126 * fr + 0.7152 * fg + 0.0722 * fb;

    fr = fr * (1.0f - grayingFactor) + luminance * grayingFactor;
    fg = fg * (1.0f - grayingFactor) + luminance * grayingFactor;
    fb = fb * (1.0f - grayingFactor) + luminance * grayingFactor;

    fr *= (1.0f - darkeningFactor);
    fg *= (1.0f - darkeningFactor);
    fb *= (1.0f - darkeningFactor);

    fr = max(0.0f, min(1.0f, fr));
    fg = max(0.0f, min(1.0f, fg));
    fb = max(0.0f, min(1.0f, fb));

    uint16_t new_r = static_cast<uint16_t>(fr * 31.0f + 0.5f);
    uint16_t new_g = static_cast<uint16_t>(fg * 63.0f + 0.5f);
    uint16_t new_b = static_cast<uint16_t>(fb * 31.0f + 0.5f);

    return (new_r << 11) | (new_g << 5) | new_b;
}

void prepareTemperaturePlot() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    float zero_diff_mapped = mapf(0, MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
    int zero_diff_y = PLOT_Y_START + PLOT_HEIGHT - (int)zero_diff_mapped;

    tft.drawFastHLine(PLOT_X_START, zero_diff_y, PLOT_WIDTH, PLOT_ZERO_COLOR);
}

void plotVoltageData() {
    if (PLOT_WIDTH <= 1) return;

    float min_voltage = 1000.0;
    float max_voltage = -1000.0;
    bool first_valid_voltage = true;

    for (int i = 0; i < PLOT_WIDTH; i++) {
        if (!isnan(voltage_values[i])) {
            if (first_valid_voltage) {
                min_voltage = voltage_values[i];
                max_voltage = voltage_values[i];
                first_valid_voltage = false;
            } else {
                min_voltage = fmin(min_voltage, voltage_values[i]);
                max_voltage = fmax(max_voltage, voltage_values[i]);
            }
        }
    }

    min_voltage = fmax(min_voltage, 1.15f);
    max_voltage = fmin(max_voltage, 3.0f);

    if (min_voltage == max_voltage) {
        if (first_valid_voltage) {
            min_voltage = 0.5f;
            max_voltage = 1.0f;
        } else {
            float offset = 0.1f;
            min_voltage -= offset;
            max_voltage += offset;
            min_voltage = fmax(min_voltage, 0.5f);
            max_voltage = fmin(max_voltage, 3.0f);
        }
    }

    uint16_t grid_color = darkerColor(GRAPH_COLOR_VOLTAGE, 0.25f);

    int grid_x = PLOT_X_START + PLOT_WIDTH / 8;

    float target_voltages[] = {min_voltage, 1.25f, 1.38f, 1.55f, max_voltage};
    int num_targets = sizeof(target_voltages) / sizeof(target_voltages[0]);

    tft.setTextColor(grid_color);
    tft.setTextSize(1);

    for (int i = 0; i < num_targets; i++) {
        float voltage = target_voltages[i];
        float mapped_y = mapf(voltage, min_voltage, max_voltage, 0, PLOT_HEIGHT);
        int grid_y = PLOT_Y_START + PLOT_HEIGHT - (int)mapped_y;

        tft.drawFastHLine(grid_x, grid_y, PLOT_WIDTH, grid_color);
        tft.drawFloat(voltage, 2, grid_x - 5, grid_y + 8, 1);
    }

    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(voltage_values[i]) && !isnan(voltage_values[i + 1])) {
            int y_voltage_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i], min_voltage, max_voltage, 0, PLOT_HEIGHT);
            int y_voltage_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(voltage_values[i + 1], min_voltage, max_voltage, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_voltage_prev, PLOT_X_START + i + 1, y_voltage_current, GRAPH_COLOR_VOLTAGE);
        }
    }

    tft.setTextColor(GRAPH_COLOR_VOLTAGE);
    tft.setTextSize(1);
    tft.drawFloat(min_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START + PLOT_HEIGHT - 15, 1);
    tft.drawFloat(max_voltage, 2, PLOT_X_START + PLOT_WIDTH - 40, PLOT_Y_START, 1);
}

void plotTemperatureData() {
    for (int i = 0; i < PLOT_WIDTH - 1; i++) {
        if (!isnan(temp1_values[i]) && !isnan(temp1_values[i + 1])) {
            int y1_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y1_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp1_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y1_prev, PLOT_X_START + i + 1, y1_current, GRAPH_COLOR_1);
        }
        if (!isnan(temp2_values[i]) && !isnan(temp2_values[i + 1])) {
            int y2_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            int y2_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(temp2_values[i + 1], MIN_TEMP, MAX_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y2_prev, PLOT_X_START + i + 1, y2_current, GRAPH_COLOR_2);
        }
        if (!isnan(diff_values[i]) && !isnan(diff_values[i + 1])) {
            int y_diff_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            int y_diff_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(diff_values[i + 1], MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_diff_prev, PLOT_X_START + i + 1, y_diff_current, GRAPH_COLOR_DIFF);
        }
        if (!isnan(current_values[i]) && !isnan(current_values[i + 1])) {
            int y_current_prev = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            int y_current_current = PLOT_Y_START + PLOT_HEIGHT - (int)mapf(current_values[i + 1], MIN_CURRENT, MAX_CURRENT, 0, PLOT_HEIGHT);
            tft.drawLine(PLOT_X_START + i, y_current_prev, PLOT_X_START + i + 1, y_current_current, GRAPH_COLOR_CURRENT);
        }
    }
}

void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    int label_line_height = 8;
    extern ThermistorSensor thermistorSensor;

    // T1 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1: ");
    if (!isnan(temp1)) {
        tft.printf("%.2f C", temp1);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);
    tft.print(" R");

    // Voltage Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START);
    tft.print("V: ");
    if (!isnan(voltage)) {
        tft.printf("%.3f V", voltage);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_VOLTAGE, TFT_BLACK);
    tft.print(" Y");

    // Current Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START+label_line_height * 1);
    tft.print("I: ");
    if (!isnan(current)) {
        tft.printf("%.3f A", current);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_CURRENT, TFT_BLACK);
    tft.print(" M");

    // VCC Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 0);
    tft.print("VCC:");
    if (!isnan(thermistorSensor.getVCC())) {
        tft.printf("%.2f mV", thermistorSensor.getVCC());
    } else {
        tft.print("Error");
    }

    // T2 Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("T2: ");
    if (!isnan(temp2)) {
        tft.printf("%.2f C", temp2);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);
    tft.print(" G");

    // Raw Millivolts Label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 1);
    tft.print("mV :");
    if (!isnan(t1_millivolts)) {
        tft.printf("%.2f mV", t1_millivolts);
    } else {
        tft.print("Error");
    }

     // duty cycle label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 2);
    tft.print("%  :");
    if (!isnan(dutyCycle)) {
        tft.printf("%u  ", dutyCycle);
    } else {
        tft.print("Error");
    }

    // Diff Label
    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("dT: ");
    if (!isnan(tempDiff)) {
        tft.printf("%.2f C", tempDiff);
    } else {
        tft.print("Error");
    }
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);
    tft.print(" B");
}


//------------------------auto label placing log graph

constexpr int GRID_SIZE = 18;
using ScoreType = int8_t;

std::pair<int,int> pixelToGrid(int x, int y, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    if (x < marginX || x >= marginX + plotAreaWidth || y < marginYTop || y >= marginYTop + plotAreaHeight) {
        return {-1, -1};
    }
    int row = static_cast<int>((static_cast<float>(y - marginYTop) / static_cast<float>(plotAreaHeight)) * GRID_SIZE);
    int col = static_cast<int>((static_cast<float>(x - marginX) / static_cast<float>(plotAreaWidth)) * GRID_SIZE);
    row = std::min(GRID_SIZE - 1, std::max(0, row));
    col = std::min(GRID_SIZE - 1, std::max(0, col));
    return {row, col};
}

std::pair<int,int> gridToPixel(int row, int col, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    float cellHeight = static_cast<float>(plotAreaHeight) / GRID_SIZE;
    float cellWidth  = static_cast<float>(plotAreaWidth)  / GRID_SIZE;
    int y = static_cast<int>(marginYTop + (row + 0.5f) * cellHeight);
    int x = static_cast<int>(marginX    + (col + 0.5f) * cellWidth);
    return {x, y};
}

int calculateVerticalPosition(int yInitial, int textHeight, LabelVerticalPlacement placement) {
    switch (placement) {
        case LabelVerticalPlacement::CENTER:
            return yInitial - textHeight / 2;
        case LabelVerticalPlacement::ABOVE:
            return yInitial - textHeight - 2;
        case LabelVerticalPlacement::BELOW:
            return yInitial + 2;
        default:
            return yInitial - textHeight / 2;
    }
}

void drawChargePlot(bool autoscaleX, bool autoscaleY) {
    extern std::vector<ChargeLogData> chargeLog;

    if (chargeLog.empty()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(6); // MC_DATUM
        tft.drawString("No charge data to plot", tft.width() / 2, tft.height() / 2, 2);
        return;
    }

    tft.fillScreen(TFT_BLACK);

    int plotAreaWidth  = tft.width() - 40;
    int plotAreaHeight = tft.height() - 60;
    int marginX        = 20;
    int marginYTop     = 20;

    unsigned long startTime = chargeLog.front().timestamp;
    unsigned long endTime   = chargeLog.back().timestamp;
    double timeScaleX = 1.0;
    if (autoscaleX && endTime > startTime) {
        timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
    } else if (!autoscaleX) {
        unsigned long window = 10UL * 60UL * 1000UL;
        long long maybeStart = static_cast<long long>(endTime) - static_cast<long long>(window);
        if (maybeStart > static_cast<long long>(startTime)) startTime = static_cast<unsigned long>(maybeStart);
        if (endTime > startTime) {
            timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
        } else {
            timeScaleX = 1.0;
        }
    }

    float currentMin = 1000.0f, currentMax = -1000.0f;
    float voltageMin = 1000.0f, voltageMax = -1000.0f;
    float dutyCycleMin = 1000.0f, dutyCycleMax = -1000.0f;
    float tempDiffMin = 1000.0f, tempDiffMax = -1000.0f;
    float estTempDiffThresholdMin = 1000.0f, estTempDiffThresholdMax = -1000.0f;
    float irLU_Min = 1000.0f, irLU_Max = -1000.0f;
    float irPairs_Min = 1000.0f, irPairs_Max = -1000.0f;

    if (autoscaleY) {
        for (const auto &logEntry : chargeLog) {
            currentMin = std::fmin(currentMin, logEntry.current);
            currentMax = std::fmax(currentMax, logEntry.current);
            voltageMin = std::fmin(voltageMin, logEntry.voltage);
            voltageMax = std::fmax(voltageMax, logEntry.voltage);
            dutyCycleMin = std::fmin(dutyCycleMin, static_cast<float>(logEntry.dutyCycle));
            dutyCycleMax = std::fmax(dutyCycleMax, static_cast<float>(logEntry.dutyCycle));
            float currentTempDiff = logEntry.batteryTemperature - logEntry.ambientTemperature;
            tempDiffMin = std::fmin(tempDiffMin, currentTempDiff);
            tempDiffMax = std::fmax(tempDiffMax, currentTempDiff);
            float estimatedDiff = currentTempDiff + 3.0f;
            float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            estTempDiffThresholdMin = std::fmin(estTempDiffThresholdMin, currentTempDiff);
            estTempDiffThresholdMax = std::fmax(estTempDiffThresholdMax, thresholdValue * 0.5f);
            irLU_Min   = std::fmin(irLU_Min,   logEntry.internalResistancePairs);
            irLU_Max   = std::fmax(irLU_Max,   logEntry.internalResistanceLoadedUnloaded);
            irPairs_Min = std::fmin(irPairs_Min, logEntry.internalResistancePairs);
            irPairs_Max = std::fmax(irPairs_Max, logEntry.internalResistanceLoadedUnloaded);
        }

        auto padIfEqual = [](float &a, float &b, float pad) {
            if (a >= b) { a -= pad; b += pad; }
        };
        padIfEqual(currentMin, currentMax, 0.1f);
        padIfEqual(voltageMin, voltageMax, 0.1f);
        padIfEqual(dutyCycleMin, dutyCycleMax, 1.0f);
        padIfEqual(tempDiffMin, tempDiffMax, 0.1f);
        padIfEqual(estTempDiffThresholdMin, estTempDiffThresholdMax, 0.1f);
        padIfEqual(irLU_Min, irLU_Max, 0.01f);
        padIfEqual(irPairs_Min, irPairs_Max, 0.01f);
    } else {
        currentMin = 0.0f; currentMax = 0.4f;
        voltageMin = 1.0f; voltageMax = 2.0f;
        dutyCycleMin = 0.0f; dutyCycleMax = 255.0f;
        tempDiffMin = -0.5f; tempDiffMax = 1.5f;
        estTempDiffThresholdMin = -0.5f; estTempDiffThresholdMax = 1.5f;
        irLU_Min = 0.0f; irLU_Max = 1.5f;
        irPairs_Min = 0.0f; irPairs_Max = 1.5f;
    }

    float scaleY = static_cast<float>(plotAreaHeight);
    auto scaleValue = [&](float val, float minVal, float maxVal) -> int {
        if (maxVal <= minVal) return marginYTop + static_cast<int>(scaleY/2.0f);
        float normalized = (val - minVal) / (maxVal - minVal);
        float pixelFromTop = (1.0f - normalized) * scaleY;
        return marginYTop + static_cast<int>(std::round(pixelFromTop));
    };
    auto inverseScaleValue = [&](int yPixel, float minVal, float maxVal) -> float {
        if (maxVal <= minVal) return (minVal + maxVal) * 0.5f;
        float normalizedY = static_cast<float>(yPixel - marginYTop) / scaleY;
        return minVal + (1.0f - normalizedY) * (maxVal - minVal);
    };

    Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE> grid;
    grid.setZero();

    tft.drawLine(marginX, marginYTop, marginX + plotAreaWidth, marginYTop, TFT_DARKGREY);
    tft.drawLine(marginX, marginYTop, marginX, marginYTop + plotAreaHeight, TFT_DARKGREY);

    auto timeToX = [&](unsigned long t) -> int {
        if (endTime <= startTime) return marginX;
        double dt = static_cast<double>(static_cast<long long>(t) - static_cast<long long>(startTime));
        double xp = static_cast<double>(marginX) + dt * timeScaleX;
        return static_cast<int>(std::round(xp));
    };

    auto drawLineAndUpdateGrid = [&](int x0, int y0, int x1, int y1, uint16_t color) {
        tft.drawLine(x0, y0, x1, y1, color);
        int dx = std::abs(x1 - x0);
        int sx = (x0 < x1) ? 1 : -1;
        int dy = -std::abs(y1 - y0);
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;
        int x = x0;
        int y = y0;
        while (true) {
            auto g = pixelToGrid(x, y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
            if (g.first != -1) {
                if (grid(g.first, g.second) < std::numeric_limits<ScoreType>::max()) {
                    grid(g.first, g.second) = static_cast<ScoreType>(grid(g.first, g.second) + 1);
                }
            }
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    };

    for (size_t i = 2; i < chargeLog.size(); ++i) {
        int x1 = timeToX(chargeLog[i - 1].timestamp);
        int x2 = timeToX(chargeLog[i].timestamp);
        {
            int y1 = scaleValue(chargeLog[i - 1].current, currentMin, currentMax);
            int y2 = scaleValue(chargeLog[i].current, currentMin, currentMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_MAGENTA);
        }
        {
            int y1 = scaleValue(chargeLog[i - 1].voltage, voltageMin, voltageMax);
            int y2 = scaleValue(chargeLog[i].voltage, voltageMin, voltageMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_YELLOW);
        }
        {
            int y1 = scaleValue(static_cast<float>(chargeLog[i - 1].dutyCycle), dutyCycleMin, dutyCycleMax);
            int y2 = scaleValue(static_cast<float>(chargeLog[i].dutyCycle), dutyCycleMin, dutyCycleMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_DARKGREY);
        }
        {
            float currentTempDiffPrev = chargeLog[i - 1].batteryTemperature - chargeLog[i - 1].ambientTemperature;
            float currentTempDiffCurr = chargeLog[i].batteryTemperature     - chargeLog[i].ambientTemperature;
            int y1 = scaleValue(currentTempDiffPrev, tempDiffMin, tempDiffMax);
            int y2 = scaleValue(currentTempDiffCurr, tempDiffMin, tempDiffMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_BLUE);
        }
        {
            size_t prevIdx = i - 1;
            size_t prevPrevIdx = (i >= 2) ? (i - 2) : prevIdx;
            float estimatedDiffPrev = estimateTempDiff(
                chargeLog[prevIdx].voltage,
                chargeLog[prevIdx].voltage,
                chargeLog[prevIdx].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[prevIdx].ambientTemperature,
                chargeLog[prevIdx].timestamp,
                chargeLog[prevPrevIdx].timestamp,
                chargeLog[prevIdx].batteryTemperature,
                0.015f, 1000.0f, 0.001477f, 4.4f, 0.9f
            );
            float estimatedDiffCurr = estimateTempDiff(
                chargeLog[i].voltage,
                chargeLog[i].voltage,
                chargeLog[i].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[i].ambientTemperature,
                chargeLog[i].timestamp,
                chargeLog[i - 1].timestamp,
                chargeLog[i].batteryTemperature,
                0.015f, 1000.0f, 0.001477f, 4.4f, 0.9f
            );
            float thresholdValuePrev = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffPrev;
            float thresholdValueCurr = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffCurr;
            int y1 = scaleValue(thresholdValuePrev, estTempDiffThresholdMin, estTempDiffThresholdMax);
            int y2 = scaleValue(thresholdValueCurr, estTempDiffThresholdMin, estTempDiffThresholdMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_RED);
        }
        {
            int y1 = scaleValue(chargeLog[i - 1].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            int y2 = scaleValue(chargeLog[i].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_ORANGE);
        }
        {
            int y1 = scaleValue(chargeLog[i - 1].internalResistancePairs, irPairs_Min, irPairs_Max);
            int y2 = scaleValue(chargeLog[i].internalResistancePairs, irPairs_Min, irPairs_Max);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_CYAN);
        }
    }

    std::vector<Label> labels;
    float labelLineLengthFactor = 0.10f;
    const int LABEL_LINE_LENGTH = 30;
    int maxLabelLineLength = static_cast<int>(plotAreaWidth * 0.1f);
    float darkeningFactor = 0.0f;
    int labelPadding = 3;
    int labelMarkingScore = 5;
    int labelDisplayThreshold = 4;

    auto createLabel = [&](const std::string& name, float value, float minValue, float maxValue, uint16_t color) {
        float scaledYValue = scaleValue(value, minValue, maxValue);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        std::string text = ss.str();

        uint16_t darkColor = color;
        if (darkeningFactor > 0) { darkColor = darkerColor(color, darkeningFactor); }

        int tw = tft.textWidth(text.c_str(), 1);
        int th = tft.fontHeight(1);

        Label label;
        label.text = text;

        float dataRange = maxValue - minValue;
        float normalizedY = (value - minValue) / dataRange;
        int plotYPixel = marginYTop + static_cast<int>((1.0f - normalizedY) * plotAreaHeight);

        label.y = plotYPixel;
        label.color = darkColor;
        label.textWidth = tw;
        label.textHeight = th;
        label.lineY = plotYPixel;
        label.lineStartX = 0;
        label.lineEndX = 0;
        label.y_initial = plotYPixel;
        label.minValue = minValue;
        label.maxValue = maxValue;

        if (plotYPixel < marginYTop + 30) {
            label.verticalPlacement = LabelVerticalPlacement::BELOW;
        } else if (plotYPixel > marginYTop + plotAreaHeight - 30) {
            label.verticalPlacement = LabelVerticalPlacement::ABOVE;
        } else {
            label.verticalPlacement = LabelVerticalPlacement::CENTER;
        }
        return label;
    };

    labels.push_back(createLabel("Current", currentMax, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Current", currentMin, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Voltage", voltageMax, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("Voltage", voltageMin, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("DutyCycle", dutyCycleMax, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("DutyCycle", dutyCycleMin, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("TempDiff", tempDiffMax, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("TempDiff", tempDiffMin, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMax, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMin, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED));
    labels.push_back(createLabel("IrLU", irLU_Max, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrLU", irLU_Min, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrPairs", irPairs_Max, irPairs_Min, irPairs_Max, TFT_CYAN));
    labels.push_back(createLabel("IrPairs", irPairs_Min, irPairs_Min, irPairs_Max, TFT_CYAN));

    auto canPlaceLabel = [&](const Label& label, int gridRow, int gridCol) {
        std::pair<int, int> gridPixelPos = gridToPixel(gridRow, gridCol, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        int gridPixelXCenter = gridPixelPos.first;
        int gridPixelYCenter = gridPixelPos.second;
        int textX;
        int textY;

        if (gridCol < GRID_SIZE / 2) {
            textX = gridPixelXCenter + LABEL_LINE_LENGTH + 2;
        } else {
            textX = gridPixelXCenter - label.textWidth - LABEL_LINE_LENGTH - 2;
        }

        if (gridRow < GRID_SIZE / 2) {
            textY = gridPixelYCenter;
        } else {
            textY = gridPixelYCenter - label.textHeight;
        }

        int labelTop = textY - labelPadding;
        int labelBottom = textY + label.textHeight + labelPadding;
        int labelLeft = textX - labelPadding;
        int labelRight = textX + label.textWidth + labelPadding;

        int startRow = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int endRow   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int startCol = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
        int endCol   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

        for (int r = startRow; r <= endRow; ++r)
            for (int c = startCol; c <= endCol; ++c)
                if (grid(r, c) > labelDisplayThreshold) return false;

        return true;
    };

    auto markLabelAsPlaced = [&](const Label& label, int gridRow, int gridCol) {
        int labelTop = label.y;
        int labelBottom = label.y + label.textHeight;
        int labelLeft = label.x;
        int labelRight = label.x + label.textWidth;

        int startRow = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int endRow   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int startCol = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
        int endCol   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

        for (int r = startRow; r <= endRow; ++r) {
            for (int c = startCol; c <= endCol; ++c) {
                if (r >= 0 && r < GRID_SIZE && c >= 0 && c < GRID_SIZE) {
                    int sum = static_cast<int>(grid(r, c)) + labelMarkingScore;
                    grid(r, c) = static_cast<ScoreType>(std::min(127, sum));
                }
            }
        }
    };

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    auto placeLabelAndLine = [&](Label& label, bool tryLeftFirst) -> bool {
        LabelVerticalPlacement verticalPlacement = label.verticalPlacement;
        int textY = calculateVerticalPosition(label.y_initial, label.textHeight, verticalPlacement);
        label.y = textY;

        std::pair<int,int> initialGridPos = pixelToGrid(marginX, label.y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        if (initialGridPos.first == -1) return false;

        int startRow = initialGridPos.first;
        bool placed = false;
        int searchRadius = 0;

        while (!placed && searchRadius < GRID_SIZE / 2) {
            int currentRowUp = startRow - searchRadius;
            int currentRowDown = startRow + searchRadius;

            auto tryRow = [&](int row) -> bool {
                if (row < 0 || row >= GRID_SIZE) return false;
                int startColLeft = 0;
                int endColLeft = GRID_SIZE / 2 - 1;
                int startColRight = GRID_SIZE / 2;
                int endColRight = GRID_SIZE - 1;

                auto tryPlacementInRow = [&](int col) -> bool {
                    std::pair<int, int> gridPixelPos = gridToPixel(row, col, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
                    int gridPixelX = gridPixelPos.first;
                    int gridPixelYCenter = gridPixelPos.second;
                    int textX;
                    int newTextY;

                    if (col < GRID_SIZE / 2) {
                        textX = gridPixelX + LABEL_LINE_LENGTH + 2;
                        label.lineStartX = gridPixelX;
                        label.lineEndX = textX - 2;
                        newTextY = gridPixelYCenter;
                    } else {
                        textX = gridPixelX - label.textWidth - LABEL_LINE_LENGTH - 2;
                        label.lineStartX = gridPixelX;
                        label.lineEndX = textX + label.textWidth + 2;
                        newTextY = gridPixelYCenter - label.textHeight / 2;
                    }

                    label.x = textX;
                    label.lineY = gridPixelPos.second;

                    int originalLabelY = label.y;
                    label.y = newTextY;
                    bool canPlace = canPlaceLabel(label, row, col);
                    label.y = originalLabelY;

                    if (canPlace) {
                        float newDataValue = inverseScaleValue(gridPixelYCenter, label.minValue, label.maxValue);
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << newDataValue;
                        label.text = ss.str();
                        label.y = newTextY;

                        tft.setTextColor(label.color);
                        tft.drawString(label.text.c_str(), textX, newTextY, 1);
                        tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);

                        markLabelAsPlaced(label, row, col);
                        placed = true;
                        return true;
                    }
                    return false;
                };

                if (tryLeftFirst) {
                    for (int col = startColLeft; col <= endColLeft; ++col) if (tryPlacementInRow(col)) return true;
                    for (int col = endColRight; col >= startColRight; --col) if (tryPlacementInRow(col)) return true;
                } else {
                    for (int col = endColRight; col >= startColRight; --col) if (tryPlacementInRow(col)) return true;
                    for (int col = startColLeft; col <= endColLeft; ++col) if (tryPlacementInRow(col)) return true;
                }
                return false;
            };

            if (tryRow(currentRowUp)) return true;
            if (currentRowDown < GRID_SIZE && currentRowDown != currentRowUp) {
                if (tryRow(currentRowDown)) return true;
            }
            ++searchRadius;
        }

        return false;
    };

    for (auto &label : labels) {
        placeLabelAndLine(label, true);
    }

    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(TL_DATUM);
    if (autoscaleX && endTime > startTime) {
        for (int i = 0; i <= 5; ++i) {
            unsigned long timePoint = startTime + (endTime - startTime) * i / 5;
            time_t t = timePoint / 1000;
            struct tm* tm_info = localtime(&t);
            char buffer[6];
            strftime(buffer, sizeof(buffer), "%H:%M", tm_info);
            int x = marginX + plotAreaWidth * i / 5;
            tft.drawLine(x, marginYTop, x, marginYTop - 5, TFT_DARKGREY);
            tft.drawString(buffer, x, marginYTop - 20, 1);
        }
    }

    int legendY = marginYTop + plotAreaHeight + 10;
    int legendX = marginX;
    int colorSize = 9;
    int textOffset = 15;
    tft.setTextDatum(TL_DATUM);
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_MAGENTA);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("I", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_YELLOW);
    tft.drawString("V", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_DARKGREY);
    tft.drawString("%", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_BLUE);
    tft.drawString("dT", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_RED);
    tft.drawString("dT/", legendX + textOffset, legendY, 1);

    legendX = marginX;
    legendY += 12;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_ORANGE);
    tft.drawString("iR(L/UL)", legendX + textOffset, legendY, 1);
    legendX += 90;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_CYAN);
    tft.drawString("iR(Pairs)", legendX + textOffset, legendY, 1);
}

#ifdef DEBUG_LABELS
#include <iostream>
int testGraph() {
    // Create some dummy data for testing
    for (int i = 0; i < 100; ++i) {
        chargeLog.push_back({(unsigned long)i * 1000, 0.2f + 0.1f * sin(i * 0.1f), 1.5f + 0.2f * cos(i * 0.05f), (uint8_t)(i % 256), 25.0f + 0.5f * i, 20.0f + 0.3f * i, 0.1f + 0.01f * i, 0.2f + 0.02f * i});
    }
    drawChargePlot(true, true);
    return 0;
}
#endif // #ifdef DEBUG_LABELS