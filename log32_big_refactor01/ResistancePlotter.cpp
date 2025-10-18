#include "ResistancePlotter.h"
#include "DisplayUtils.h"

ResistancePlotter::ResistancePlotter(TFT_eSPI& tft) : tft(tft) {}

namespace { // Anonymous namespace to limit scope of helper functions

struct DataPoint {
    float current;
    float resistance;
};

std::vector<DataPoint> extractValidData(const float (*rawData)[2], int dataCount) {
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

void plotResistanceData(TFT_eSPI& tft, const std::vector<DataPoint>& dataPoints, uint16_t color, float minCurrent, float maxCurrent, float minResistance, float maxResistance, int graphXStart, int graphYStart, int graphXEnd, int graphYEnd) {
    tft.setTextColor(color);
    if (dataPoints.size() >= 2) {
        for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
            float current1 = dataPoints[i].current;
            float resistance1 = dataPoints[i].resistance;
            float current2 = dataPoints[i + 1].current;
            float resistance2 = dataPoints[i + 1].resistance;

            int x1 = DisplayUtils::mapf(current1, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y1 = DisplayUtils::mapf(resistance1, minResistance, maxResistance, graphYEnd, graphYStart);

            int x2 = DisplayUtils::mapf(current2, minCurrent, maxCurrent, graphXStart, graphXEnd);
            int y2 = DisplayUtils::mapf(resistance2, minResistance, maxResistance, graphYEnd, graphYStart);

            tft.drawLine(x1, y1, x2, y2, color);
        }
    }
}

} // namespace

void ResistancePlotter::draw(const DataStore& data) {
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);

    auto validResistanceData       = extractValidData(data.internalResistanceData, data.resistanceDataCount);
    auto validResistanceDataPairs  = extractValidData(data.internalResistanceDataPairs, data.resistanceDataCountPairs);

    if (validResistanceData.size() < 2 && validResistanceDataPairs.size() < 2) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("Not enough valid data");
        return;
    }

    float minCurrent     = FLT_MAX, maxCurrent     = -FLT_MAX;
    float minResistance  = FLT_MAX, maxResistance  = -FLT_MAX;
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
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(PLOT_X_START + 20, PLOT_Y_START + PLOT_HEIGHT / 2 - 10);
        tft.println("No valid data to plot");
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

    const int AXIS_MARGIN_LEFT   = 50;
    const int AXIS_MARGIN_BOTTOM = 30;
    const int AXIS_MARGIN_TOP    = 20;
    const int AXIS_MARGIN_RIGHT  = 20;

    const int graphXStart  = PLOT_X_START + AXIS_MARGIN_LEFT;
    const int graphYStart  = PLOT_Y_START + AXIS_MARGIN_TOP;
    const int graphXEnd    = PLOT_X_START + PLOT_WIDTH - AXIS_MARGIN_RIGHT;
    const int graphYEnd    = PLOT_Y_START + PLOT_HEIGHT - AXIS_MARGIN_BOTTOM;
    const int graphWidth   = graphXEnd - graphXStart;
    const int graphHeight  = graphYEnd - graphYStart;

    tft.drawLine(graphXStart, graphYEnd, graphXEnd, graphYEnd, PLOT_X_AXIS_COLOR);
    tft.drawLine(graphXStart, graphYEnd, graphXStart, graphYStart, PLOT_Y_AXIS_COLOR);

    plotResistanceData(tft, validResistanceData, GRAPH_COLOR_RESISTANCE,
                       minCurrent, maxCurrent, minResistance, maxResistance,
                       graphXStart, graphYStart, graphXEnd, graphYEnd);

    plotResistanceData(tft, validResistanceDataPairs, GRAPH_COLOR_RESISTANCE_PAIR,
                       minCurrent, maxCurrent, minResistance, maxResistance,
                       graphXStart, graphYStart, graphXEnd, graphYEnd);

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(graphXStart + graphWidth / 2 - 25, graphYEnd + 12);
    tft.print("Current (A)");

    tft.setCursor(PLOT_X_START + 5, graphYStart + graphHeight / 2 - 5);
    tft.print("R (Ω)");

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

    auto drawRegression = [&](float slope, float intercept,
                              int color, const char *label,
                              float minCurr, float maxCurr,
                              bool pair = false) {
        float rMin = slope * minCurr + intercept;
        float rMax = slope * maxCurr + intercept;

        if ((rMin >= minResistance && rMin <= maxResistance) ||
            (rMax >= minResistance && rMax <= maxResistance)) {

            int y1 = DisplayUtils::mapf(rMin, minResistance, maxResistance, graphYEnd, graphYStart);
            int y2 = DisplayUtils::mapf(rMax, minResistance, maxResistance, graphYEnd, graphYStart);

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

    if (data.resistanceDataCount >= 2)
        drawRegression(data.regressedInternalResistanceSlope, data.regressedInternalResistanceIntercept,
                       TFT_WHITE, "Rint(LU)", minCurrent, maxCurrent, false);
    else if (data.resistanceDataCount == 1)
        tft.drawCircle(DisplayUtils::mapf(data.internalResistanceData[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd),
                       DisplayUtils::mapf(data.internalResistanceData[0][1], minResistance, maxResistance, graphYEnd, graphYStart),
                       2, TFT_WHITE);

    if (data.resistanceDataCountPairs >= 2)
        drawRegression(data.regressedInternalResistancePairsSlope, data.regressedInternalResistancePairsIntercept,
                       TFT_GREEN, "Rint(Pair)", minCurrent, maxCurrent, true);
    else if (data.resistanceDataCountPairs == 1)
        tft.drawCircle(DisplayUtils::mapf(data.internalResistanceDataPairs[0][0], minCurrent, maxCurrent, graphXStart, graphXEnd),
                       DisplayUtils::mapf(data.internalResistanceDataPairs[0][1], minResistance, maxResistance, graphYEnd, graphYStart),
                       2, TFT_GREEN);

    const int legendX = graphXStart + 10;
    const int legendY = PLOT_Y_START + 5;
    const int legendSpacing = 12;

    tft.setTextSize(1);

    tft.setTextColor(GRAPH_COLOR_RESISTANCE);
    tft.setCursor(legendX, legendY);
    tft.print("R_int(1)");

    tft.setTextColor(GRAPH_COLOR_RESISTANCE_PAIR);
    tft.setCursor(legendX, legendY + legendSpacing);
    tft.print("R_int(2)");
}