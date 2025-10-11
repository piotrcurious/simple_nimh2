#ifndef GRAPHING_H
#define GRAPHING_H

#include <TFT_eSPI.h>
#include <vector>
#include <string>
#include "logging.h" // For ChargeLogData

// --- Enums and Structs for Graphing ---

enum class LabelVerticalPlacement {
    CENTER,
    ABOVE,
    BELOW
};

struct Label {
    std::string text;
    int x;
    int y;
    uint16_t color;
    int textWidth;
    int textHeight;
    int lineStartX;
    int lineEndX;
    int y_initial;
    int lineY;
    LabelVerticalPlacement verticalPlacement;
    float minValue;
    float maxValue;
};

struct TickLabel {
    int y;
    float value;
    uint16_t color;
    Label label;
};

// --- Function Declarations for Graphing ---

void drawChargePlot(bool autoscaleX, bool autoscaleY);
void displayInternalResistanceGraph();
void plotVoltageData();
void plotTemperatureData();
void prepareTemperaturePlot();
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, int x, int y, int width, int height);
void drawDutyCycleBar(int low, int high, int mid, float current, float threshold);
void bigUglyMessage(const String& measurementType = "");
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
float mapf(float value, float in_min, float in_max, float out_min, float out_max);
uint16_t darkerColor(uint16_t color, float darkeningFactor);

#ifdef DEBUG_LABELS
int testGraph();
#endif // DEBUG_LABELS

#endif // GRAPHING_H