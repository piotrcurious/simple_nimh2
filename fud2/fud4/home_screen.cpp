#include "home_screen.h"
#include "definitions.h"
#include <math.h>
#include <algorithm>
#include <cstring>
#include <stdint.h>


HomeScreen::HomeScreen()
    : lastRenderMs(0), lastGatherMs(0)
{
}

void HomeScreen::begin() {
    for (size_t i = 0; i < PLOT_WIDTH; ++i) {
        temp_history[i] = NAN;
        humidity_history[i] = NAN;
    }
}

void HomeScreen::shiftHistoryLeft(float *arr, size_t len) noexcept {
    if (len <= 1) return;
    std::memmove(arr, arr + 1, (len - 1) * sizeof(float));
    arr[len - 1] = NAN;
}

void HomeScreen::gatherData() {
    unsigned long now = millis();
    if (now - lastGatherMs < GATHER_INTERVAL_MS) return;
    lastGatherMs = now;

    WEB_LOCK();
    shiftHistoryLeft(temp_history, PLOT_WIDTH);
    shiftHistoryLeft(humidity_history, PLOT_WIDTH);

    float t = sht4Sensor.getTemperature();
    float h = sht4Sensor.getHumidity();

    temp_history[PLOT_WIDTH - 1] = t;
    humidity_history[PLOT_WIDTH - 1] = h;
    WEB_UNLOCK();
}

double HomeScreen::calculateDewPoint(double temperature, double humidity) {
    constexpr double a = 17.27;
    constexpr double b = 237.7;
    double h = humidity;
    if (!isfinite(h) || h <= 0.0) h = 0.0001;
    if (h > 100.0) h = 100.0;
    double t = temperature;
    if (!isfinite(t)) t = 25.0;
    double alpha = ((a * t) / (b + t)) + log(h / 100.0);
    double dew_point = (b * alpha) / (a - alpha);
    if (!isfinite(dew_point)) return t;
    return dew_point;
}
