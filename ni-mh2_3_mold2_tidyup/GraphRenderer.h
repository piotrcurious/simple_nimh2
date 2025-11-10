#ifndef GRAPHRENDERER_H
#define GRAPHRENDERER_H

#include "GraphDataManager.h"
#include <TFT_eSPI.h>

class GraphRenderer {
public:
    GraphRenderer();

    void drawGraph(const GraphDataManager* dataManager);

private:
    void drawCompressedGraph(const GraphDataManager* dataManager, bool isTemp);
    void drawRawGraph(const GraphDataManager* dataManager, bool isTemp);
    void drawGridAndAxes(const GraphDataManager* dataManager);

    // Helper function to map data values to screen coordinates
    static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    // Color definitions
    static constexpr uint16_t TEMP_COLOR = TFT_RED;
    static constexpr uint16_t HUMIDITY_COLOR = TFT_BLUE;
    static constexpr uint16_t GRID_COLOR = TFT_DARKGREY;
};

#endif // GRAPHRENDERER_H
