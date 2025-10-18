#ifndef MAIN_PLOTTER_H
#define MAIN_PLOTTER_H

#include "Shared.h"
#include "DataStore.h"

class MainPlotter {
public:
    MainPlotter(TFT_eSPI& tft);
    void draw(const DataStore& data);

private:
    TFT_eSPI& tft;
};

#endif // MAIN_PLOTTER_H