#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "Shared.h"
#include "DataStore.h"

class DisplayManager {
public:
    DisplayManager();
    void begin();
    void update(const DataStore& data);

private:
    TFT_eSPI tft;
};

#endif // DISPLAY_MANAGER_H