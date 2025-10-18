#ifndef RESISTANCE_PLOTTER_H
#define RESISTANCE_PLOTTER_H

#include "Shared.h"
#include "DataStore.h"

class ResistancePlotter {
public:
    ResistancePlotter(TFT_eSPI& tft);
    void draw(const DataStore& data);

private:
    TFT_eSPI& tft;
};

#endif // RESISTANCE_PLOTTER_H