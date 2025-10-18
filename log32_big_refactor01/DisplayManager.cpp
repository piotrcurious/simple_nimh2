#include "DisplayManager.h"
#include "MainPlotter.h"
#include "ChargePlotter.h"
#include "ResistancePlotter.h"

DisplayManager::DisplayManager() : tft(TFT_eSPI()) {}

void DisplayManager::begin() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
}

void DisplayManager::update(const DataStore& data) {
    switch (data.currentDisplayState) {
        case DISPLAY_STATE_MAIN:
            {
                MainPlotter plotter(tft);
                plotter.draw(data);
            }
            break;
        case DISPLAY_STATE_CHARGE_GRAPH:
            {
                ChargePlotter plotter(tft);
                plotter.draw(data);
            }
            break;
        case DISPLAY_STATE_IR_GRAPH:
            {
                ResistancePlotter plotter(tft);
                plotter.draw(data);
            }
            break;
    }
}