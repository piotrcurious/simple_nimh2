#include "ChargePlotter.h"
#include "DisplayUtils.h"

ChargePlotter::ChargePlotter(TFT_eSPI& tft) : tft(tft) {}

void ChargePlotter::draw(const DataStore& data) {
    if (data.chargeLog.empty()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(6);
        tft.drawString("No charge data to plot", tft.width() / 2, tft.height() / 2, 2);
        return;
    }

    tft.fillScreen(TFT_BLACK);

    int plotAreaWidth  = tft.width() - 2;
    int plotAreaHeight = tft.height() - 25;
    int marginX        = 0;
    int marginYTop     = 10;

    unsigned long startTime = data.chargeLog.front().timestamp;
    unsigned long endTime   = data.chargeLog.back().timestamp;
    double timeScaleX = 1.0;
    if (endTime > startTime) {
        timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
    }

    float currentMin = 1000.0f, currentMax = -1000.0f;
    float voltageMin = 1000.0f, voltageMax = -1000.0f;
    float dutyCycleMin = 1000.0f, dutyCycleMax = -1000.0f;
    float tempDiffMin = 1000.0f, tempDiffMax = -1000.0f;
    float estTempDiffThresholdMin = 1000.0f, estTempDiffThresholdMax = -1000.0f;
    float irLU_Min = 1000.0f, irLU_Max = -1000.0f;
    float irPairs_Min = 1000.0f, irPairs_Max = -1000.0f;

    for (const auto &logEntry : data.chargeLog) {
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

    float scaleY = static_cast<float>(plotAreaHeight);
    auto scaleValue = [&](float val, float minVal, float maxVal) -> int {
        if (maxVal <= minVal) return marginYTop + static_cast<int>(scaleY/2.0f);
        float normalized = (val - minVal) / (maxVal - minVal);
        float pixelFromTop = (1.0f - normalized) * scaleY;
        return marginYTop + static_cast<int>(std::round(pixelFromTop));
    };

    tft.drawLine(marginX, marginYTop, marginX + plotAreaWidth, marginYTop, TFT_DARKGREY);
    tft.drawLine(marginX, marginYTop, marginX, marginYTop + plotAreaHeight, TFT_DARKGREY);

    auto timeToX = [&](unsigned long t) -> int {
        if (endTime <= startTime) return marginX;
        double dt = static_cast<double>(static_cast<long long>(t) - static_cast<long long>(startTime));
        double xp = static_cast<double>(marginX) + dt * timeScaleX;
        return static_cast<int>(std::round(xp));
    };

    for (size_t i = 1; i < data.chargeLog.size(); ++i) {
        int x1 = timeToX(data.chargeLog[i - 1].timestamp);
        int x2 = timeToX(data.chargeLog[i].timestamp);

        {
            int y1 = scaleValue(data.chargeLog[i - 1].current, currentMin, currentMax);
            int y2 = scaleValue(data.chargeLog[i].current, currentMin, currentMax);
            tft.drawLine(x1, y1, x2, y2, TFT_MAGENTA);
        }
        {
            int y1 = scaleValue(data.chargeLog[i - 1].voltage, voltageMin, voltageMax);
            int y2 = scaleValue(data.chargeLog[i].voltage, voltageMin, voltageMax);
            tft.drawLine(x1, y1, x2, y2, TFT_YELLOW);
        }
        {
            int y1 = scaleValue(static_cast<float>(data.chargeLog[i - 1].dutyCycle), dutyCycleMin, dutyCycleMax);
            int y2 = scaleValue(static_cast<float>(data.chargeLog[i].dutyCycle), dutyCycleMin, dutyCycleMax);
            tft.drawLine(x1, y1, x2, y2, TFT_DARKGREY);
        }
        {
            float currentTempDiffPrev = data.chargeLog[i - 1].batteryTemperature - data.chargeLog[i - 1].ambientTemperature;
            float currentTempDiffCurr = data.chargeLog[i].batteryTemperature     - data.chargeLog[i].ambientTemperature;
            int y1 = scaleValue(currentTempDiffPrev, tempDiffMin, tempDiffMax);
            int y2 = scaleValue(currentTempDiffCurr, tempDiffMin, tempDiffMax);
            tft.drawLine(x1, y1, x2, y2, TFT_BLUE);
        }
        {
            int y1 = scaleValue(data.chargeLog[i - 1].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            int y2 = scaleValue(data.chargeLog[i].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            tft.drawLine(x1, y1, x2, y2, TFT_ORANGE);
        }
        {
            int y1 = scaleValue(data.chargeLog[i - 1].internalResistancePairs, irPairs_Min, irPairs_Max);
            int y2 = scaleValue(data.chargeLog[i].internalResistancePairs, irPairs_Min, irPairs_Max);
            tft.drawLine(x1, y1, x2, y2, TFT_CYAN);
        }
    }
}