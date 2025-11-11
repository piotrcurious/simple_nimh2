#include "home_screen.h"
#include "GraphRenderer.h" // Will be created in the next step
#include "definitions.h"
#include <math.h>

HomeScreen::HomeScreen()
    : lastRenderMs(0), lastGatherMs(0),
      current_temp(NAN), current_humidity(NAN), current_dew_point(NAN)
{
    dataManager = new GraphDataManager();
    renderer = new GraphRenderer();
}

HomeScreen::~HomeScreen() {
    delete dataManager;
    delete renderer;
}

void HomeScreen::begin() {
    dataManager->begin();
}

void HomeScreen::gatherData() {
    unsigned long now = millis();
    if (now - lastGatherMs < GATHER_INTERVAL_MS) return;
    lastGatherMs = now;

    // read sensors
    float t = sht4Sensor.getTemperature();
    float h = sht4Sensor.getHumidity();

    // store current values for display labels
    current_temp = t;
    current_humidity = h;
    current_dew_point = static_cast<float>(calculateDewPoint(t, h));

    // log data for graphing
    dataManager->logData(t, h, now);
}

void HomeScreen::render() {
    unsigned long now = millis();
    if (now - lastRenderMs < RENDER_INTERVAL_MS) return;
    lastRenderMs = now;

    // The renderer now handles clearing the plot area, so no need for fillRect here.
    renderer->drawGraph(dataManager);
    drawLabels();
}

void HomeScreen::drawLabels() {
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // current temperature (left)
    tft.setTextColor(TFT_RED, TFT_BLACK);
    if (!isValidSample(current_temp)) {
        tft.drawString("-- C", PLOT_X_START, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(current_temp, 2) + " C", PLOT_X_START, PLOT_Y_START + 12);
    }

    // current humidity (to the right)
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    if (!isValidSample(current_humidity)) {
        tft.drawString("-- %", PLOT_X_START + 90, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(current_humidity, 2) + " %", PLOT_X_START + 90, PLOT_Y_START + 12);
    }

    // dew point (smaller, under temperature)
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    if (!isValidSample(current_dew_point)) {
        tft.drawString("Dew: -- C", PLOT_X_START, PLOT_Y_START + 30);
    } else {
        tft.drawString("Dew: " + String(current_dew_point, 2) + " C", PLOT_X_START, PLOT_Y_START + 30);
    }
}

double HomeScreen::calculateDewPoint(double temperature, double humidity) {
    // Magnus-Tetens formula (valid for typical temperature/humidity ranges)
    constexpr double a = 17.27;
    constexpr double b = 237.7;
    // protect against invalid humidity input
    double h = humidity;
    if (!isfinite(h) || h <= 0.0) h = 0.0001;    // avoid log(0) / division issues
    if (h > 100.0) h = 100.0;
    double alpha = ((a * temperature) / (b + temperature)) + log(h / 100.0);
    double dew_point = (b * alpha) / (a - alpha);
    return dew_point;
}
