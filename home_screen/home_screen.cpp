#include "home_screen.h"

HomeScreen::HomeScreen() {
    last_render_time = 0;
    last_data_gather_time = 0;
}

void HomeScreen::begin() {
    for (int i = 0; i < SCREEN_WIDTH; i++) {
        temp_history[i] = 25.0f;
        humidity_history[i] = 50.0f;
        dew_point_history[i] = 15.0f;
    }
}

void HomeScreen::render() {
    unsigned long now = millis();
    if (now - last_render_time >= 1000) { // Render every second
        last_render_time = now;
        tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
        drawGraph();
        tft.fillRect(0, PLOT_Y_START + PLOT_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT - PLOT_HEIGHT, TFT_BLACK);
        drawLabels();
    }
}

void HomeScreen::gatherData() {
    unsigned long now = millis();
    if (now - last_data_gather_time >= 60000) { // Gather data every minute
        last_data_gather_time = now;
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
    tft.setTextSize(2);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, LABEL_Y_START);
    tft.printf("Temp: %.2f C", temp_history[SCREEN_WIDTH - 1]);

    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.setCursor(10, LABEL_Y_START + 20);
    tft.printf("Humi: %.2f %%", humidity_history[SCREEN_WIDTH - 1]);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, LABEL_Y_START + 40);
    tft.printf("Dew:  %.2f C", dew_point_history[SCREEN_WIDTH - 1]);

    if (last_mAh_charged > 0) {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(10, LABEL_Y_START + 60);
        tft.printf("Last Charge: %.2f mAh", last_mAh_charged);
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
