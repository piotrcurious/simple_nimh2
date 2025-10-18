#ifndef DATAPLOTTER_H
#define DATAPLOTTER_H

#include <TFT_eSPI.h>
#include "definitions.h"
#include <vector>
#include <string>
#include <utility>
#include <ArduinoEigenDense.h>

// Forward declaration
class ThermistorSensor;

class DataPlotter {
public:
    DataPlotter(TFT_eSPI& tft);

    void prepareTemperaturePlot();
    void plotVoltageData();
    void plotTemperatureData();
    void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current, ThermistorSensor& thermistorSensor, uint8_t dutyCycle);
    void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);
    void displayInternalResistanceGraph(float (*internalResistanceData)[2], int resistanceDataCount, float (*internalResistanceDataPairs)[2], int resistanceDataCountPairs, float regressedInternalResistanceSlope, float regressedInternalResistanceIntercept, float regressedInternalResistancePairsSlope, float regressedInternalResistancePairsIntercept);
    void drawChargePlot(bool autoscaleX, bool autoscaleY, const std::vector<ChargeLogData>& chargeLog, float regressedInternalResistancePairsIntercept);
    void bigUglyMessage(const String& measurementType);
    void printThermistorSerial(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);


private:
    TFT_eSPI& tft;

    float temp1_values[PLOT_WIDTH];
    float temp2_values[PLOT_WIDTH];
    float diff_values[PLOT_WIDTH];
    float voltage_values[PLOT_WIDTH];
    float current_values[PLOT_WIDTH];
    float MAX_DIFF_TEMP = 1.5;

    static float mapf(float value, float in_min, float in_max, float out_min, float out_max);
    uint16_t darkerColor(uint16_t color, float darkeningFactor);

    // Charge plot helpers
    std::pair<int,int> pixelToGrid(int x, int y, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    std::pair<int,int> gridToPixel(int row, int col, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    int calculateVerticalPosition(int yInitial, int textHeight, LabelVerticalPlacement placement);
    void drawLineAndUpdateGrid(Eigen::Matrix<int8_t, 18, 18>& grid, int x0, int y0, int x1, int y1, uint16_t color, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    bool canPlaceLabel(const Label& label, const Eigen::Matrix<int8_t, 18, 18>& grid, int gridRow, int gridCol, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    void markLabelAsPlaced(Eigen::Matrix<int8_t, 18, 18>& grid, const Label& label, int gridRow, int gridCol, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    bool placeLabelAndLine(Label& label, Eigen::Matrix<int8_t, 18, 18>& grid, bool tryLeftFirst, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop);
    Label createLabel(const std::string& name, float value, float minValue, float maxValue, uint16_t color, int plotAreaWidth, int plotAreaHeight, int marginYTop);

    float estimateTempDiff(float v_start, float v_end, float current, float internal_resistance,
                       float ambient_temp, unsigned long t_start, unsigned long t_end,
                       float last_batt_temp, float cell_mass_kg, float specific_heat,
                       float surface_area_m2, float convective_h, float emissivity);

};

#endif // DATAPLOTTER_H