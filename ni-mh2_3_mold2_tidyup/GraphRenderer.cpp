#include "GraphRenderer.h"
#include "definitions.h"
#include <cmath>

// Helper function to evaluate a polynomial, needed for rendering
static double evaluatePolynomial(const float *coefficients, uint8_t degree, double tNorm) {
    double result = 0.0;
    double tPower = 1.0;
    for (int i = 0; i <= degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= tNorm;
    }
    return result;
}

GraphRenderer::GraphRenderer() {}

inline float GraphRenderer::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (isnan(x) || isinf(x)) return out_min; // Should be safe
    if (in_max == in_min) return (out_min + out_max) * 0.5f;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void GraphRenderer::drawGraph(const GraphDataManager* dataManager) {
    drawGridAndAxes(dataManager);

    // Draw Temperature (compressed and raw)
    drawCompressedGraph(dataManager, true);
    drawRawGraph(dataManager, true);

    // Draw Humidity (compressed and raw)
    drawCompressedGraph(dataManager, false);
    drawRawGraph(dataManager, false);

    // Redraw border on top
    tft.drawRect(PLOT_X_START - 1, PLOT_Y_START - 1, PLOT_WIDTH + 2, PLOT_HEIGHT + 2, TFT_WHITE);
}

void GraphRenderer::drawGridAndAxes(const GraphDataManager* dataManager) {
    // --- Draw Grid ---
    for (int g = 0; g <= 4; ++g) {
        int gy = PLOT_Y_START + ((PLOT_HEIGHT * g) / 4);
        tft.drawFastHLine(PLOT_X_START, gy, PLOT_WIDTH, GRID_COLOR);
    }
    for (int gx = PLOT_X_START + PLOT_WIDTH; gx >= PLOT_X_START; gx -= 60) {
        tft.drawFastVLine(gx, PLOT_Y_START, PLOT_HEIGHT, GRID_COLOR);
    }

    // --- Draw Axis Labels ---
    tft.setTextSize(1);
    // Left axis (Temperature)
    tft.setTextColor(TEMP_COLOR, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(String(dataManager->getMaxValue(true), 1) + "C", PLOT_X_START, PLOT_Y_START + 1);
    tft.setTextDatum(BL_DATUM);
    tft.drawString(String(dataManager->getMinValue(true), 1) + "C", PLOT_X_START, PLOT_Y_START + PLOT_HEIGHT -1);

    // Right axis (Humidity)
    tft.setTextColor(HUMIDITY_COLOR, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String(dataManager->getMaxValue(false), 0) + "%", PLOT_X_START + PLOT_WIDTH, PLOT_Y_START + 1);
    tft.setTextDatum(BR_DATUM);
    tft.drawString(String(dataManager->getMinValue(false), 0) + "%", PLOT_X_START + PLOT_WIDTH, PLOT_Y_START + PLOT_HEIGHT -1);
}

void GraphRenderer::drawRawGraph(const GraphDataManager* dataManager, bool isTemp) {
    const uint16_t count = dataManager->getRawDataCount();
    if (count < 2) return;

    const float* data = isTemp ? dataManager->getRawTempHistory() : dataManager->getRawHumidityHistory();
    const uint32_t* timestamps = dataManager->getRawTimestamps();
    const uint16_t color = isTemp ? TEMP_COLOR : HUMIDITY_COLOR;
    const float y_min = dataManager->getMinValue(isTemp);
    const float y_max = dataManager->getMaxValue(isTemp);

    uint32_t min_ts = timestamps[0];
    uint32_t max_ts = timestamps[count - 1];

    for (uint16_t i = 0; i < count - 1; ++i) {
        if (!isnan(data[i]) && !isnan(data[i+1])) {
            int16_t x1 = mapFloat(timestamps[i], min_ts, max_ts, PLOT_X_START, PLOT_X_START + PLOT_WIDTH -1);
            int16_t y1 = mapFloat(data[i], y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);
            int16_t x2 = mapFloat(timestamps[i+1], min_ts, max_ts, PLOT_X_START, PLOT_X_START + PLOT_WIDTH -1);
            int16_t y2 = mapFloat(data[i+1], y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);
            tft.drawLine(x1, y1, x2, y2, color);
        }
    }
}


void GraphRenderer::drawCompressedGraph(const GraphDataManager* dataManager, bool isTemp) {
    const uint8_t segmentCount = dataManager->getSegmentCount();
    if (segmentCount == 0) return;

    const PolynomialSegment* segments = isTemp ? dataManager->getTempSegments() : dataManager->getHumiditySegments();
    const uint16_t color = isTemp ? TEMP_COLOR : HUMIDITY_COLOR;
    const float y_min = dataManager->getMinValue(isTemp);
    const float y_max = dataManager->getMaxValue(isTemp);

    const uint32_t* raw_timestamps = dataManager->getRawTimestamps();
    const uint16_t raw_data_count = dataManager->getRawDataCount();
    if (raw_data_count < 1) return;

    uint32_t windowEnd = raw_timestamps[raw_data_count - 1];
    uint32_t windowStart = raw_timestamps[0];
    uint32_t xMax = windowEnd - dataManager->getRawTimeDelta();

    // Calculate the width of the screen available for the compressed graph
    float compression_ratio = (float)(xMax - windowStart) / (float)(windowEnd - windowStart);
    int compressed_width = PLOT_WIDTH * compression_ratio;

    int16_t last_y = -1;

    // Start drawing from the right edge of the compressed area
    for (int x_screen = PLOT_X_START + compressed_width; x_screen >= PLOT_X_START; --x_screen) {
        // Map screen x to a timestamp in the compressed range
        uint32_t t_target = mapFloat(x_screen, PLOT_X_START, PLOT_X_START + compressed_width, windowStart, xMax);

        // Find which polynomial segment this timestamp falls into
        uint32_t t_cursor = xMax;
        int seg_idx = segmentCount - 1;
        int poly_idx = dataManager->getCurrentPolyIndex() -1;
        if (poly_idx < 0) poly_idx = POLY_COUNT_PER_SEGMENT -1;

        bool found = false;
        while (seg_idx >= 0) {
            uint32_t poly_delta = segments[seg_idx].timeDeltas[poly_idx];
            if (t_target >= t_cursor - poly_delta && t_target <= t_cursor) {
                // We found the right polynomial
                double t_norm = (double)(t_target - (t_cursor - poly_delta)) / poly_delta;
                float y_val = evaluatePolynomial(segments[seg_idx].coefficients[poly_idx], POLY_DEGREE, t_norm);
                int16_t y_screen = mapFloat(y_val, y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);

                if (last_y != -1) {
                    tft.drawLine(x_screen, y_screen, x_screen + 1, last_y, color);
                } else {
                    tft.drawPixel(x_screen, y_screen, color);
                }
                last_y = y_screen;
                found = true;
                break;
            }
            t_cursor -= poly_delta;
            poly_idx--;
            if (poly_idx < 0) {
                seg_idx--;
                poly_idx = POLY_COUNT_PER_SEGMENT -1;
            }
        }
        if(!found) last_y = -1;
    }
}
