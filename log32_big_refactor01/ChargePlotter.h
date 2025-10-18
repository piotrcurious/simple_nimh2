#ifndef CHARGE_PLOTTER_H
#define CHARGE_PLOTTER_H

#include "Shared.h"
#include "DataStore.h"

class ChargePlotter {
public:
    ChargePlotter(TFT_eSPI& tft);
    void draw(const DataStore& data);

private:
    TFT_eSPI& tft;
};

#endif // CHARGE_PLOTTER_H