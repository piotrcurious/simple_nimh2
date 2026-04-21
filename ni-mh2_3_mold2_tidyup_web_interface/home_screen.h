#ifndef HOME_SCREEN_H
#define HOME_SCREEN_H

#include "definitions.h"
#include <Arduino.h>

class HomeScreen {
public:
    HomeScreen();
    void begin();
    void gatherData();
    void render() {} // No longer used for TFT

    static constexpr int GATHER_INTERVAL_MS = 1000;
    static constexpr int RENDER_INTERVAL_MS = 1000;

    float temp_history[PLOT_WIDTH];
    float humidity_history[PLOT_WIDTH];
    float dew_point_history[PLOT_WIDTH];

private:
    unsigned long lastRenderMs;
    unsigned long lastGatherMs;

    void shiftHistoryLeft(float *arr, size_t len) noexcept;
    inline bool isValidSample(float x) const { return !std::isnan(x); }
    double calculateDewPoint(double temperature, double humidity);
};

extern HomeScreen homeScreen;

#endif // HOME_SCREEN_H
