#ifndef HOME_SCREEN_H
#define HOME_SCREEN_H

#include "definitions.h"

class HomeScreen {
public:
    HomeScreen();
    void begin();
    void render();
    void gatherData();

private:
    void drawGraph();
    void drawLabels();
    double calculateDewPoint(double temperature, double humidity);

    float temp_history[SCREEN_WIDTH];
    float humidity_history[SCREEN_WIDTH];
    float dew_point_history[SCREEN_WIDTH];
    unsigned long last_render_time;
    unsigned long last_data_gather_time;
};

#endif // HOME_SCREEN_H
