#pragma once
#include <Arduino.h>
#include <math.h>
#include <cstring> // memmove
#include "definitions.h"

class HomeScreen {
public:
    HomeScreen();

    void begin();
    void render();       // call frequently (non-blocking)
    void gatherData();   // call frequently (non-blocking)

private:
    // timing (ms)
    static constexpr unsigned long RENDER_INTERVAL_MS = 2000UL;
    static constexpr unsigned long GATHER_INTERVAL_MS = 60000UL;

    // dash/grid constants
    static constexpr int TIME_GRID_STEP_PX = 60; // pixels between vertical lines
    static constexpr int DASH_LEN_PX = 3;        // dash length for vertical grid
    static constexpr int GAP_LEN_PX = 3;         // gap length for vertical grid
    static constexpr int GRID_LINES = 4;         // horizontal grid lines

    unsigned long lastRenderMs;
    unsigned long lastGatherMs;

    float temp_history[SCREEN_WIDTH];
    float humidity_history[SCREEN_WIDTH];
    float dew_point_history[SCREEN_WIDTH];

    // helpers
    static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    static inline bool isValidSample(float v) { return !isnan(v) && isfinite(v); }
    void shiftHistoryLeft(float *arr, size_t len) noexcept;

    void drawGraph();
    void drawSeries(const float *history, size_t len,
                    float plot_min, float plot_max,
                    uint16_t color, bool rightAxis = false);
    void drawGrid(const float *humidity_history, size_t history_len, float h_plot_min, float h_plot_max);
    void drawAxisLabels(float td_plot_min, float td_plot_max,
                        float h_plot_min, float h_plot_max);
    void drawLabels();
    float interpolatedHumidityAtPixel(const float *history, size_t len, int px, float &outHumidity);
    double calculateDewPoint(double temperature, double humidity);
};
