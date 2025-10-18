#include "DataPlotter.h"
#include "ThermistorSensor.h"
#include <cfloat>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <numeric>
#include <ctime>
#include <esp_timer.h>


DataPlotter::DataPlotter(TFT_eSPI& tft) : tft(tft) {
    for (int i = 0; i < PLOT_WIDTH; ++i) {
        temp1_values[i] = NAN;
        temp2_values[i] = NAN;
        diff_values[i] = NAN;
        voltage_values[i] = NAN;
        current_values[i] = NAN;
    }
}

void DataPlotter::printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current) {
    Serial.print("Thermistor 1 (Top, SHT4x): ");
    if (isnan(temp1)) Serial.print("Error"); else Serial.printf("%.3f °C", temp1);
    Serial.print(", Thermistor 2 (Bottom): ");
    if (isnan(temp2)) Serial.print("Error"); else Serial.printf("%.3f °C", temp2);
    Serial.print(", Diff (T1-T2): ");
    if (isnan(tempDiff)) Serial.print("Error"); else Serial.printf("%.3f °C", tempDiff);
    Serial.printf(", Voltage : %.3f V", voltage);
    Serial.printf(", Current : %.3f A", current);
    Serial.println(" °C");
}


float DataPlotter::mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DataPlotter::bigUglyMessage(const String& measurementType) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
    tft.print(measurementType);
}

void DataPlotter::updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current) {
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

uint16_t DataPlotter::darkerColor(uint16_t color, float darkeningFactor) {
    darkeningFactor = std::max(0.0f, std::min(1.0f, darkeningFactor));
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

    fr = std::max(0.0f, std::min(1.0f, fr));
    fg = std::max(0.0f, std::min(1.0f, fg));
    fb = std::max(0.0f, std::min(1.0f, fb));

    uint16_t new_r = static_cast<uint16_t>(fr * 31.0f + 0.5f);
    uint16_t new_g = static_cast<uint16_t>(fg * 63.0f + 0.5f);
    uint16_t new_b = static_cast<uint16_t>(fb * 31.0f + 0.5f);

    return (new_r << 11) | (new_g << 5) | new_b;
}

void DataPlotter::prepareTemperaturePlot() {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    float zero_diff_mapped = mapf(0, MIN_DIFF_TEMP, MAX_DIFF_TEMP, 0, PLOT_HEIGHT);
    int zero_diff_y = PLOT_Y_START + PLOT_HEIGHT - (int)zero_diff_mapped;

    tft.drawFastHLine(PLOT_X_START, zero_diff_y, PLOT_WIDTH, PLOT_ZERO_COLOR);
}

void DataPlotter::plotVoltageData() {
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
        if (voltage != min_voltage && voltage != max_voltage) {
            tft.drawFloat(voltage, 2, grid_x - 5, grid_y + 8, 1);
        }
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

void DataPlotter::plotTemperatureData() {
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

void DataPlotter::displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current, ThermistorSensor& thermistorSensor, uint8_t dutyCycle) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(LABEL_TEXT_SIZE);
    int label_line_height = 8;

    tft.setCursor(PLOT_X_START, LABEL_Y_START);
    tft.print("T1: ");
    if (!isnan(temp1)) tft.printf("%.2f C", temp1); else tft.print("Error");
    tft.setTextColor(GRAPH_COLOR_1, TFT_BLACK);
    tft.print(" R");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START);
    tft.print("V: ");
    if (!isnan(voltage)) tft.printf("%.3f V", voltage); else tft.print("Error");
    tft.setTextColor(GRAPH_COLOR_VOLTAGE, TFT_BLACK);
    tft.print(" Y");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(PLOT_X_START + 100, LABEL_Y_START+label_line_height * 1);
    tft.print("I: ");
    if (!isnan(current)) tft.printf("%.3f A", current); else tft.print("Error");
    tft.setTextColor(GRAPH_COLOR_CURRENT, TFT_BLACK);
    tft.print(" M");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 0);
    tft.print("VCC:");
    if (!isnan(thermistorSensor.getVCC())) tft.printf("%.2f mV", thermistorSensor.getVCC()); else tft.print("Error");

    tft.setCursor(PLOT_X_START, LABEL_Y_START + label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("T2: ");
    if (!isnan(temp2)) tft.printf("%.2f C", temp2); else tft.print("Error");
    tft.setTextColor(GRAPH_COLOR_2, TFT_BLACK);
    tft.print(" G");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 1);
    tft.print("mV :");
    if (!isnan(t1_millivolts)) tft.printf("%.2f mV", t1_millivolts); else tft.print("Error");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(PLOT_X_START + 260, LABEL_Y_START + label_line_height * 2);
    tft.print("%  :");
    if (!isnan(dutyCycle)) tft.printf("%u  ", dutyCycle); else tft.print("Error");

    tft.setCursor(PLOT_X_START, LABEL_Y_START + 2 * label_line_height);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("dT: ");
    if (!isnan(tempDiff)) tft.printf("%.2f C", tempDiff); else tft.print("Error");
    tft.setTextColor(GRAPH_COLOR_DIFF, TFT_BLACK);
    tft.print(" B");
}

namespace {
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

void plotResistanceData(TFT_eSPI& tft, const std::vector<DataPoint>& dataPoints, uint16_t color, float minCurrent, float maxCurrent, float minResistance, float maxResistance, int graphXStart, int graphYStart, int graphXEnd, int graphYEnd, float (*mapf_func)(float, float, float, float, float)) {
    tft.setTextColor(color);
    if (dataPoints.size() >= 2) {
        for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
            float current1 = dataPoints[i].current;
            float resistance1 = dataPoints[i].resistance;
            float current2 = dataPoints[i + 1].current;
            float resistance2 = dataPoints[i + 1].resistance;

            int x1 = mapf_func(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = mapf_func(resistance1, minResistance, maxResistance, graphYEnd, graphYStart);

            int x2 = mapf_func(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = mapf_func(resistance2, minResistance, maxResistance, graphYEnd, graphYStart);

            tft.drawLine(x1, y1, x2, y2, color);
        }
    }
}
}

void DataPlotter::displayInternalResistanceGraph(float (*internalResistanceData)[2], int resistanceDataCount, float (*internalResistanceDataPairs)[2], int resistanceDataCountPairs, float regressedInternalResistanceSlope, float regressedInternalResistanceIntercept, float regressedInternalResistancePairsSlope, float regressedInternalResistancePairsIntercept) {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    auto validResistanceData = extractValidData(internalResistanceData, resistanceDataCount);
    auto validResistanceDataPairs = extractValidData(internalResistanceDataPairs, resistanceDataCountPairs);

    if (validResistanceData.size() < 2 && validResistanceDataPairs.size() < 2) {
        bigUglyMessage("Not enough valid data");
        return;
    }

    float minCurrent = FLT_MAX, maxCurrent = -FLT_MAX;
    float minResistance = FLT_MAX, maxResistance = -FLT_MAX;
    bool dataFound = false;

    if (!validResistanceData.empty()) {
        findMinMax(validResistanceData, minCurrent, maxCurrent, minResistance, maxResistance);
        dataFound = true;
    }
    if (!validResistanceDataPairs.empty()) {
        findMinMax(validResistanceDataPairs, minCurrent, maxCurrent, minResistance, maxResistance);
        dataFound = true;
    }

    if (!dataFound) {
        bigUglyMessage("No valid data to plot");
        return;
    }

    auto adjustRange = [](float &minVal, float &maxVal, float paddingFactor) {
        float range = maxVal - minVal;
        if (range > 0) {
            minVal -= range * paddingFactor;
            maxVal += range * paddingFactor;
        } else {
            minVal -= 0.1f;
            maxVal += 0.1f;
        }
    };
    adjustRange(minCurrent, maxCurrent, 0.1f);
    adjustRange(minResistance, maxResistance, 0.05f);

    const int AXIS_MARGIN_LEFT = 50, AXIS_MARGIN_BOTTOM = 30, AXIS_MARGIN_TOP = 20, AXIS_MARGIN_RIGHT = 20;
    const int graphXStart = PLOT_X_START + AXIS_MARGIN_LEFT, graphYStart = PLOT_Y_START + AXIS_MARGIN_TOP;
    const int graphXEnd = PLOT_X_START + PLOT_WIDTH - AXIS_MARGIN_RIGHT, graphYEnd = PLOT_Y_START + PLOT_HEIGHT - AXIS_MARGIN_BOTTOM;
    const int graphWidth = graphXEnd - graphXStart, graphHeight = graphYEnd - graphYStart;

    tft.drawLine(graphXStart, graphYEnd, graphXEnd, graphYEnd, PLOT_X_AXIS_COLOR);
    tft.drawLine(graphXStart, graphYEnd, graphXStart, graphYStart, PLOT_Y_AXIS_COLOR);

    plotResistanceData(tft, validResistanceData, GRAPH_COLOR_RESISTANCE, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd, &DataPlotter::mapf);
    plotResistanceData(tft, validResistanceDataPairs, GRAPH_COLOR_RESISTANCE_PAIR, minCurrent, maxCurrent, minResistance, maxResistance, graphXStart, graphYStart, graphXEnd, graphYEnd, &DataPlotter::mapf);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(graphXStart + graphWidth / 2 - 25, graphYEnd + 12);
    tft.print("Current (A)");
    tft.setCursor(PLOT_X_START + 5, graphYStart + graphHeight / 2 - 5);
    tft.print("R (O)");

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(graphXStart - 45, graphYEnd - 8);
    tft.printf("%.2f", minResistance);
    tft.setCursor(graphXStart - 45, graphYStart - 8);
    tft.printf("%.2f", maxResistance);
    tft.setCursor(graphXStart - 5, graphYEnd + 5);
    tft.printf("%.2fA", minCurrent);
    tft.setCursor(graphXEnd - 40, graphYEnd + 5);
    tft.printf("%.2fA", maxCurrent);

    auto drawRegression = [&](float slope, float intercept, int color, const char *label, float minCurr, float maxCurr, bool pair = false) {
        float rMin = slope * minCurr + intercept;
        float rMax = slope * maxCurr + intercept;
        if ((rMin >= minResistance && rMin <= maxResistance) || (rMax >= minResistance && rMax <= maxResistance)) {
            int y1 = mapf(rMin, minResistance, maxResistance, graphYEnd, graphYStart);
            int y2 = mapf(rMax, minResistance, maxResistance, graphYEnd, graphYStart);
            tft.drawLine(graphXStart, y1, graphXEnd, y2, color);
            tft.setTextColor(color);
            tft.setTextSize(1);
            char buf[50];
            snprintf(buf, sizeof(buf), "%s: %.2f + %.2f*I", label, intercept, slope);
            int labelY = constrain(y2 + (pair ? 12 : -12), graphYStart + 5, graphYEnd - 10);
            tft.setCursor(graphXEnd - 140, labelY);
            tft.print(buf);
        } else {
            tft.setTextColor(color);
            tft.setTextSize(1);
            tft.setCursor(graphXStart + 10, graphYStart + (pair ? 30 : 10));
            tft.printf("%s out of range", label);
        }
    };

    if (resistanceDataCount >= 2)
        drawRegression(regressedInternalResistanceSlope, regressedInternalResistanceIntercept, TFT_WHITE, "Rint(LU)", minCurrent, maxCurrent, false);
    else if (resistanceDataCount == 1)
        tft.drawCircle(mapf(internalResistanceData[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd), mapf(internalResistanceData[0][1], minResistance, maxResistance, graphYEnd, graphYStart), 2, TFT_WHITE);

    if (resistanceDataCountPairs >= 2)
        drawRegression(regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept, TFT_GREEN, "Rint(Pair)", minCurrent, maxCurrent, true);
    else if (resistanceDataCountPairs == 1)
        tft.drawCircle(mapf(internalResistanceDataPairs[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd), mapf(internalResistanceDataPairs[0][1], minResistance, maxResistance, graphYEnd, graphYStart), 2, TFT_GREEN);

    const int legendX = graphXStart + 10, legendY = PLOT_Y_START + 5, legendSpacing = 12;
    tft.setTextSize(1);
    tft.setTextColor(GRAPH_COLOR_RESISTANCE);
    tft.setCursor(legendX, legendY);
    tft.print("R_int(1)");
    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
    tft.setCursor(legendX, legendY + legendSpacing);
    tft.print("R_int(2)");
}

// Charge Plot Implementation
constexpr int GRID_SIZE = 18;
using ScoreType = int8_t;

void DataPlotter::drawChargePlot(bool autoscaleX, bool autoscaleY, const std::vector<ChargeLogData>& chargeLog, float regressedInternalResistancePairsIntercept) {
    if (chargeLog.empty()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(6);
        tft.drawString("No charge data to plot", tft.width() / 2, tft.height() / 2, 2);
        return;
    }
    tft.fillScreen(TFT_BLACK);
    int plotAreaWidth = tft.width() - 2, plotAreaHeight = tft.height() - 25, marginX = 0, marginYTop = 10;
    unsigned long startTime = chargeLog.front().timestamp, endTime = chargeLog.back().timestamp;
    double timeScaleX = 1.0;
    if (autoscaleX && endTime > startTime) timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
    else if (!autoscaleX) {
        unsigned long window = 10UL * 60UL * 1000UL;
        if (endTime > window) startTime = endTime - window;
        if (endTime > startTime) timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
    }

    float currentMin = 1000.0f, currentMax = -1000.0f, voltageMin = 1000.0f, voltageMax = -1000.0f, dutyCycleMin = 1000.0f, dutyCycleMax = -1000.0f;
    float tempDiffMin = 1000.0f, tempDiffMax = -1000.0f, estTempDiffThresholdMin = 1000.0f, estTempDiffThresholdMax = -1000.0f;
    float irLU_Min = 1000.0f, irLU_Max = -1000.0f, irPairs_Min = 1000.0f, irPairs_Max = -1000.0f;

    if (autoscaleY) {
        for (const auto &logEntry : chargeLog) {
            currentMin = std::fmin(currentMin, logEntry.current); currentMax = std::fmax(currentMax, logEntry.current);
            voltageMin = std::fmin(voltageMin, logEntry.voltage); voltageMax = std::fmax(voltageMax, logEntry.voltage);
            dutyCycleMin = std::fmin(dutyCycleMin, static_cast<float>(logEntry.dutyCycle)); dutyCycleMax = std::fmax(dutyCycleMax, static_cast<float>(logEntry.dutyCycle));
            float currentTempDiff = logEntry.batteryTemperature - logEntry.ambientTemperature;
            tempDiffMin = std::fmin(tempDiffMin, currentTempDiff); tempDiffMax = std::fmax(tempDiffMax, currentTempDiff);
            float estimatedDiff = currentTempDiff + 3.0f, thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            estTempDiffThresholdMin = std::fmin(estTempDiffThresholdMin, currentTempDiff); estTempDiffThresholdMax = std::fmax(estTempDiffThresholdMax, thresholdValue * 0.5f);
            irLU_Min = std::fmin(irLU_Min, logEntry.internalResistanceLoadedUnloaded); irLU_Max = std::fmax(irLU_Max, logEntry.internalResistanceLoadedUnloaded);
            irPairs_Min = std::fmin(irPairs_Min, logEntry.internalResistancePairs); irPairs_Max = std::fmax(irPairs_Max, logEntry.internalResistancePairs);
        }
        auto padIfEqual = [](float &a, float &b, float pad) { if (a >= b) { a -= pad; b += pad; } };
        padIfEqual(currentMin, currentMax, 0.1f); padIfEqual(voltageMin, voltageMax, 0.1f); padIfEqual(dutyCycleMin, dutyCycleMax, 1.0f);
        padIfEqual(tempDiffMin, tempDiffMax, 0.1f); padIfEqual(estTempDiffThresholdMin, estTempDiffThresholdMax, 0.1f);
        padIfEqual(irLU_Min, irLU_Max, 0.01f); padIfEqual(irPairs_Min, irPairs_Max, 0.01f);
    } else {
        currentMin = 0.0f; currentMax = 0.4f; voltageMin = 1.0f; voltageMax = 2.0f; dutyCycleMin = 0.0f; dutyCycleMax = 255.0f;
        tempDiffMin = -0.5f; tempDiffMax = 1.5f; estTempDiffThresholdMin = -0.5f; estTempDiffThresholdMax = 1.5f;
        irLU_Min = 0.0f; irLU_Max = 1.5f; irPairs_Min = 0.0f; irPairs_Max = 1.5f;
    }

    float scaleY = static_cast<float>(plotAreaHeight);
    auto scaleValue = [&](float val, float minVal, float maxVal) -> int {
        if (maxVal <= minVal) return marginYTop + static_cast<int>(scaleY/2.0f);
        return marginYTop + static_cast<int>(std::round((1.0f - (val - minVal) / (maxVal - minVal)) * scaleY));
    };
    auto timeToX = [&](unsigned long t) -> int {
        if (endTime <= startTime) return marginX;
        return static_cast<int>(std::round(static_cast<double>(marginX) + static_cast<double>(static_cast<long long>(t) - static_cast<long long>(startTime)) * timeScaleX));
    };

    Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE> grid; grid.setZero();
    tft.drawLine(marginX, marginYTop, marginX + plotAreaWidth, marginYTop, TFT_DARKGREY);
    tft.drawLine(marginX, marginYTop, marginX, marginYTop + plotAreaHeight, TFT_DARKGREY);

    for (size_t i = 2; i < chargeLog.size(); ++i) {
        int x1 = timeToX(chargeLog[i - 1].timestamp), x2 = timeToX(chargeLog[i].timestamp);
        drawLineAndUpdateGrid(grid, x1, scaleValue(chargeLog[i - 1].current, currentMin, currentMax), x2, scaleValue(chargeLog[i].current, currentMin, currentMax), TFT_MAGENTA, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        drawLineAndUpdateGrid(grid, x1, scaleValue(chargeLog[i - 1].voltage, voltageMin, voltageMax), x2, scaleValue(chargeLog[i].voltage, voltageMin, voltageMax), TFT_YELLOW, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        drawLineAndUpdateGrid(grid, x1, scaleValue(static_cast<float>(chargeLog[i - 1].dutyCycle), dutyCycleMin, dutyCycleMax), x2, scaleValue(static_cast<float>(chargeLog[i].dutyCycle), dutyCycleMin, dutyCycleMax), TFT_DARKGREY, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        drawLineAndUpdateGrid(grid, x1, scaleValue(chargeLog[i - 1].batteryTemperature - chargeLog[i - 1].ambientTemperature, tempDiffMin, tempDiffMax), x2, scaleValue(chargeLog[i].batteryTemperature - chargeLog[i].ambientTemperature, tempDiffMin, tempDiffMax), TFT_BLUE, plotAreaWidth, plotAreaHeight, marginX, marginYTop);

        float estDiffPrev = estimateTempDiff(chargeLog[i-1].voltage, chargeLog[i-1].voltage, chargeLog[i-1].current, regressedInternalResistancePairsIntercept, chargeLog[i-1].ambientTemperature, chargeLog[i-2].timestamp, chargeLog[i-1].timestamp, chargeLog[i-1].batteryTemperature, DEFAULT_CELL_MASS_KG, DEFAULT_SPECIFIC_HEAT, DEFAULT_SURFACE_AREA_M2, DEFAULT_CONVECTIVE_H, DEFAULT_EMISSIVITY);
        float estDiffCurr = estimateTempDiff(chargeLog[i].voltage, chargeLog[i].voltage, chargeLog[i].current, regressedInternalResistancePairsIntercept, chargeLog[i].ambientTemperature, chargeLog[i-1].timestamp, chargeLog[i].timestamp, chargeLog[i].batteryTemperature, DEFAULT_CELL_MASS_KG, DEFAULT_SPECIFIC_HEAT, DEFAULT_SURFACE_AREA_M2, DEFAULT_CONVECTIVE_H, DEFAULT_EMISSIVITY);
        drawLineAndUpdateGrid(grid, x1, scaleValue(MAX_TEMP_DIFF_THRESHOLD + estDiffPrev, estTempDiffThresholdMin, estTempDiffThresholdMax), x2, scaleValue(MAX_TEMP_DIFF_THRESHOLD + estDiffCurr, estTempDiffThresholdMin, estTempDiffThresholdMax), TFT_RED, plotAreaWidth, plotAreaHeight, marginX, marginYTop);

        drawLineAndUpdateGrid(grid, x1, scaleValue(chargeLog[i - 1].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max), x2, scaleValue(chargeLog[i].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max), TFT_ORANGE, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        drawLineAndUpdateGrid(grid, x1, scaleValue(chargeLog[i - 1].internalResistancePairs, irPairs_Min, irPairs_Max), x2, scaleValue(chargeLog[i].internalResistancePairs, irPairs_Min, irPairs_Max), TFT_CYAN, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
    }

    std::vector<Label> labels;
    labels.push_back(createLabel("Current", currentMax, currentMin, currentMax, TFT_MAGENTA, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("Current", currentMin, currentMin, currentMax, TFT_MAGENTA, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("Voltage", voltageMax, voltageMin, voltageMax, TFT_YELLOW, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("Voltage", voltageMin, voltageMin, voltageMax, TFT_YELLOW, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("DutyCycle", dutyCycleMax, dutyCycleMin, dutyCycleMax, TFT_DARKGREY, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("DutyCycle", dutyCycleMin, dutyCycleMin, dutyCycleMax, TFT_DARKGREY, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("TempDiff", tempDiffMax, tempDiffMin, tempDiffMax, TFT_BLUE, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("TempDiff", tempDiffMin, tempDiffMin, tempDiffMax, TFT_BLUE, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMax, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMin, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("IrLU", irLU_Max, irLU_Min, irLU_Max, TFT_ORANGE, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("IrLU", irLU_Min, irLU_Min, irLU_Max, TFT_ORANGE, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("IrPairs", irPairs_Max, irPairs_Min, irPairs_Max, TFT_CYAN, plotAreaWidth, plotAreaHeight, marginYTop));
    labels.push_back(createLabel("IrPairs", irPairs_Min, irPairs_Min, irPairs_Max, TFT_CYAN, plotAreaWidth, plotAreaHeight, marginYTop));

    for (auto &label : labels) placeLabelAndLine(label, grid, true, plotAreaWidth, plotAreaHeight, marginX, marginYTop);

    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(BL_DATUM);
    if (autoscaleX && endTime > startTime) {
        for (int i = 0; i <= 5; ++i) {
            unsigned long timePoint = startTime + (endTime - startTime) * i / 5;
            time_t t = timePoint / 1000; struct tm* tm_info = localtime(&t); char buffer[6]; strftime(buffer, sizeof(buffer), "%H:%M", tm_info);
            int x = marginX + plotAreaWidth * i / 5;
            tft.drawLine(x, marginYTop+2, x, marginYTop - 5, TFT_DARKGREY); tft.drawString(buffer, x, marginYTop , 1);
        }
    }
    int legendY = marginYTop + plotAreaHeight + 5, legendX = marginX, colorSize = 8, textOffset = 12;
    tft.setTextDatum(TL_DATUM);
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_MAGENTA); tft.setTextColor(TFT_MAGENTA); tft.drawString("I", legendX + textOffset, legendY, 1); legendX += 20;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_YELLOW); tft.setTextColor(TFT_YELLOW); tft.drawString("V", legendX + textOffset, legendY, 1); legendX += 20;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_DARKGREY); tft.setTextColor(TFT_DARKGREY); tft.drawString("%", legendX + textOffset, legendY, 1); legendX += 25;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_BLUE); tft.setTextColor(TFT_BLUE); tft.drawString("dT", legendX + textOffset, legendY, 1); legendX += 25;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_RED); tft.setTextColor(TFT_RED); tft.drawString("/dT", legendX + textOffset, legendY, 1); legendX += 40;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_ORANGE); tft.setTextColor(TFT_ORANGE); tft.drawString("RiMH", legendX + textOffset, legendY, 1); legendX += 40;
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_CYAN); tft.setTextColor(TFT_CYAN); tft.drawString("Ri", legendX + textOffset, legendY, 1);
}

std::pair<int,int> DataPlotter::pixelToGrid(int x, int y, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    if (x < marginX || x >= marginX + plotAreaWidth || y < marginYTop || y >= marginYTop + plotAreaHeight) return {-1, -1};
    int row = static_cast<int>((static_cast<float>(y - marginYTop) / static_cast<float>(plotAreaHeight)) * GRID_SIZE);
    int col = static_cast<int>((static_cast<float>(x - marginX) / static_cast<float>(plotAreaWidth)) * GRID_SIZE);
    return {std::min(GRID_SIZE - 1, std::max(0, row)), std::min(GRID_SIZE - 1, std::max(0, col))};
}

std::pair<int,int> DataPlotter::gridToPixel(int row, int col, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    float cellHeight = static_cast<float>(plotAreaHeight) / GRID_SIZE;
    float cellWidth  = static_cast<float>(plotAreaWidth)  / GRID_SIZE;
    return {static_cast<int>(marginX + (col + 0.5f) * cellWidth), static_cast<int>(marginYTop + (row + 0.5f) * cellHeight)};
}

int DataPlotter::calculateVerticalPosition(int yInitial, int textHeight, LabelVerticalPlacement placement) {
    switch (placement) {
        case LabelVerticalPlacement::CENTER: return yInitial - textHeight / 2;
        case LabelVerticalPlacement::ABOVE: return yInitial - textHeight - 2;
        case LabelVerticalPlacement::BELOW: return yInitial + 2;
        default: return yInitial - textHeight / 2;
    }
}

void DataPlotter::drawLineAndUpdateGrid(Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE>& grid, int x0, int y0, int x1, int y1, uint16_t color, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    tft.drawLine(x0, y0, x1, y1, color);
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    while (true) {
        auto g = pixelToGrid(x, y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        if (g.first != -1 && grid(g.first, g.second) < std::numeric_limits<ScoreType>::max()) grid(g.first, g.second)++;
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

bool DataPlotter::canPlaceLabel(const Label& label, const Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE>& grid, int gridRow, int gridCol, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    std::pair<int, int> gridPixelPos = gridToPixel(gridRow, gridCol, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
    int textX = (gridCol < GRID_SIZE / 2) ? gridPixelPos.first + 30 + 2 : gridPixelPos.first - label.textWidth - 30 - 2;
    int textY = (gridRow < GRID_SIZE / 2) ? gridPixelPos.second : gridPixelPos.second - label.textHeight;

    int labelTop = textY - 2, labelBottom = textY + label.textHeight + 2;
    int labelLeft = textX - 2, labelRight = textX + label.textWidth + 2;

    int startRow = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
    int endRow   = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
    int startCol = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
    int endCol   = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

    for (int r = startRow; r <= endRow; ++r) for (int c = startCol; c <= endCol; ++c) if (grid(r, c) > 3) return false;
    return true;
}

void DataPlotter::markLabelAsPlaced(Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE>& grid, const Label& label, int gridRow, int gridCol, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    int labelTop = label.y, labelBottom = label.y + label.textHeight;
    int labelLeft = label.x, labelRight = label.x + label.textWidth;

    int startRow = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
    int endRow   = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
    int startCol = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
    int endCol   = std::max(0, std::min(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

    for (int r = startRow; r <= endRow; ++r) for (int c = startCol; c <= endCol; ++c) grid(r, c) = static_cast<ScoreType>(std::min(127, static_cast<int>(grid(r, c)) + 4));
}

bool DataPlotter::placeLabelAndLine(Label& label, Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE>& grid, bool tryLeftFirst, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    label.y = calculateVerticalPosition(label.y_initial, label.textHeight, label.verticalPlacement);
    std::pair<int,int> initialGridPos = pixelToGrid(marginX, label.y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
    if (initialGridPos.first == -1) return false;
    int startRow = initialGridPos.first;
    for (int searchRadius = 0; searchRadius < GRID_SIZE / 2; ++searchRadius) {
        int currentRowUp = startRow - searchRadius, currentRowDown = startRow + searchRadius;
        auto tryRow = [&](int row) -> bool {
            if (row < 0 || row >= GRID_SIZE) return false;
            auto tryPlacementInRow = [&](int col) -> bool {
                std::pair<int, int> gridPixelPos = gridToPixel(row, col, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
                int textX, newTextY;
                if (col < GRID_SIZE / 2) { textX = gridPixelPos.first + 30 + 2; label.lineStartX = gridPixelPos.first; label.lineEndX = textX - 2; newTextY = gridPixelPos.second; }
                else { textX = gridPixelPos.first - label.textWidth - 30 - 2; label.lineStartX = gridPixelPos.first; label.lineEndX = textX + label.textWidth + 2; newTextY = gridPixelPos.second - label.textHeight / 2; }
                label.x = textX; label.lineY = gridPixelPos.second;
                int originalLabelY = label.y; label.y = newTextY;
                bool canPlace = canPlaceLabel(label, grid, row, col, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
                label.y = originalLabelY;
                if (canPlace) {
                    float newDataValue = (label.maxValue - label.minValue) * (1.0f - static_cast<float>(gridPixelPos.second - marginYTop) / plotAreaHeight) + label.minValue;
                    std::stringstream ss; ss << std::fixed << std::setprecision(2) << newDataValue; label.text = ss.str();
                    label.y = newTextY;
                    tft.setTextColor(label.color); tft.drawString(label.text.c_str(), textX, newTextY, 1);
                    tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);
                    markLabelAsPlaced(grid, label, row, col, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
                    return true;
                } return false;
            };
            if (tryLeftFirst) { for (int col = 0; col < GRID_SIZE / 2; ++col) if (tryPlacementInRow(col)) return true; for (int col = GRID_SIZE - 1; col >= GRID_SIZE / 2; --col) if (tryPlacementInRow(col)) return true; }
            else { for (int col = GRID_SIZE - 1; col >= GRID_SIZE / 2; --col) if (tryPlacementInRow(col)) return true; for (int col = 0; col < GRID_SIZE / 2; ++col) if (tryPlacementInRow(col)) return true; }
            return false;
        };
        if (tryRow(currentRowUp)) return true;
        if (currentRowDown < GRID_SIZE && currentRowDown != currentRowUp && tryRow(currentRowDown)) return true;
    } return false;
}

Label DataPlotter::createLabel(const std::string& name, float value, float minValue, float maxValue, uint16_t color, int plotAreaWidth, int plotAreaHeight, int marginYTop) {
    std::stringstream ss; ss << std::fixed << std::setprecision(2) << value;
    Label label;
    label.text = ss.str();
    label.color = darkerColor(color, 0.0f);
    label.textWidth = tft.textWidth(label.text.c_str(), 1);
    label.textHeight = tft.fontHeight(1);
    float normalizedY = (maxValue > minValue) ? (value - minValue) / (maxValue - minValue) : 0.5f;
    label.y_initial = marginYTop + static_cast<int>((1.0f - normalizedY) * plotAreaHeight);
    label.minValue = minValue;
    label.maxValue = maxValue;
    if (label.y_initial < marginYTop + 30) label.verticalPlacement = LabelVerticalPlacement::BELOW;
    else if (label.y_initial > marginYTop + plotAreaHeight - 30) label.verticalPlacement = LabelVerticalPlacement::ABOVE;
    else label.verticalPlacement = LabelVerticalPlacement::CENTER;
    return label;
}

float DataPlotter::estimateTempDiff(float v_start, float v_end, float current, float internal_resistance,
                       float ambient_temp, unsigned long t_start, unsigned long t_end,
                       float last_batt_temp, float cell_mass_kg, float specific_heat,
                       float surface_area_m2, float convective_h, float emissivity) {
    if (t_end <= t_start) return last_batt_temp - ambient_temp;

    float avg_v = (v_start + v_end) / 2.0f;
    float power_dissipated = current * current * internal_resistance;
    float delta_t_seconds = (t_end - t_start) / 1000.0f;
    float temp_rise_no_loss = (power_dissipated * delta_t_seconds) / (cell_mass_kg * specific_heat);
    float T_batt_prev_K = last_batt_temp + 273.15f;
    float T_amb_K = ambient_temp + 273.15f;
    float q_conv = convective_h * surface_area_m2 * (T_batt_prev_K - T_amb_K);
    float sigma = 5.670374419e-8;
    float q_rad = emissivity * sigma * surface_area_m2 * (pow(T_batt_prev_K, 4) - pow(T_amb_K, 4));
    float temp_fall_loss = ((q_conv + q_rad) * delta_t_seconds) / (cell_mass_kg * specific_heat);
    float new_batt_temp = last_batt_temp + temp_rise_no_loss - temp_fall_loss;
    return new_batt_temp - ambient_temp;
}