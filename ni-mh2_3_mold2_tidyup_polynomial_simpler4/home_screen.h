#pragma once
#include <Arduino.h>
#include <math.h>
#include <cstring>
#include "definitions.h"
#include "GraphDataManager.h"

// Forward declaration
class GraphRenderer;

class HomeScreen {
public:
    HomeScreen();
    ~HomeScreen();

    void begin();
    void render();       // call frequently (non-blocking)
    void gatherData();   // call frequently (non-blocking)

private:
    // timing (ms)
    static constexpr unsigned long RENDER_INTERVAL_MS = 2000UL;
    static constexpr unsigned long GATHER_INTERVAL_MS = 100UL;

    unsigned long lastRenderMs;
    unsigned long lastGatherMs;

    GraphDataManager* dataManager;
    GraphRenderer* renderer;

    float current_temp;
    float current_humidity;
    float current_dew_point;

    // helpers
    static inline bool isValidSample(float v) { return !isnan(v) && isfinite(v); }
    void drawLabels();
    double calculateDewPoint(double temperature, double humidity);
};
