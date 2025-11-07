#include "home_screen.h"

HomeScreen::HomeScreen() {
    last_update_time = 0;
}

void HomeScreen::begin() {
    for (int i = 0; i < SCREEN_WIDTH; i++) {
        temp_history[i] = 25.0f;
        humidity_history[i] = 50.0f;
        dew_point_history[i] = 15.0f;
    }
}

void HomeScreen::update() {
    unsigned long now = millis();
    if (now - last_update_time >= 60000) { // Update every minute
        last_update_time = now;
        updateData();
        tft.fillScreen(TFT_BLACK);
        drawGraph();
        drawLabels();
    }
}

void HomeScreen::updateData() {
    // Shift history arrays
    for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
        temp_history[i] = temp_history[i + 1];
        humidity_history[i] = humidity_history[i + 1];
        dew_point_history[i] = dew_point_history[i + 1];
    }

    // Add new data points
    temp_history[SCREEN_WIDTH - 1] = sht4Sensor.getTemperature();
    humidity_history[SCREEN_WIDTH - 1] = sht4Sensor.getHumidity();
    dew_point_history[SCREEN_WIDTH - 1] = calculateDewPoint(sht4Sensor.getTemperature(), sht4Sensor.getHumidity());
}

void HomeScreen::drawGraph() {
    // Temperature
    for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
        int y1 = map(temp_history[i], 15, 35, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        int y2 = map(temp_history[i + 1], 15, 35, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        tft.drawLine(i, y1, i + 1, y2, TFT_RED);
    }

    // Humidity
    for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
        int y1 = map(humidity_history[i], 0, 100, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        int y2 = map(humidity_history[i + 1], 0, 100, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        tft.drawLine(i, y1, i + 1, y2, TFT_BLUE);
    }

    // Dew Point
    for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
        int y1 = map(dew_point_history[i], 0, 30, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        int y2 = map(dew_point_history[i + 1], 0, 30, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
        tft.drawLine(i, y1, i + 1, y2, TFT_GREEN);
    }
}

void HomeScreen::drawLabels() {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.printf("Temp: %.2f C", temp_history[SCREEN_WIDTH - 1]);

    tft.setCursor(10, 30);
    tft.printf("Humi: %.2f %%", humidity_history[SCREEN_WIDTH - 1]);

    tft.setCursor(10, 50);
    tft.printf("Dew:  %.2f C", dew_point_history[SCREEN_WIDTH - 1]);

    if (mAh_charged > 0) {
        tft.setCursor(10, 70);
        tft.printf("Last Charge: %.2f mAh", mAh_charged);
    }
}

double HomeScreen::calculateDewPoint(double temperature, double humidity) {
    // Magnus-Tetens formula
    double a = 17.27;
    double b = 237.7;
    double alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
    double dew_point = (b * alpha) / (a - alpha);
    return dew_point;
}
