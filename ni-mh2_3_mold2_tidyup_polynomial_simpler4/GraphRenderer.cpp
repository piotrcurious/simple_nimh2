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
    const uint16_t raw_data_count = dataManager->getRawDataCount();
    if (raw_data_count < 2) {
        tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK); // Clear if no data
        drawGridAndAxes(dataManager);
        return;
    }

    const uint32_t* raw_timestamps = dataManager->getRawTimestamps();
    uint32_t windowStart, windowEnd;
    GraphDataManager::ViewMode mode = dataManager->getViewMode();

    const uint32_t latest_raw_timestamp = raw_timestamps[raw_data_count - 1];

    if (mode == GraphDataManager::VIEW_MODE_FULL) {
        uint32_t total_historical_delta = dataManager->getTotalTimeDelta();
        // The raw delta is the actual span of the raw history array
        uint32_t full_raw_delta = raw_timestamps[raw_data_count - 1] - raw_timestamps[0];
        windowEnd = latest_raw_timestamp;
        windowStart = windowEnd - total_historical_delta - full_raw_delta;
    } else { // LIVE or PANNING
        windowEnd = dataManager->getWindowEndTime();
        // The window size is constant, equivalent to the time span of the raw buffer
        uint32_t window_duration = raw_timestamps[raw_data_count - 1] - raw_timestamps[0];
        if (window_duration == 0) window_duration = 1;
        windowStart = windowEnd - window_duration;
    }
    if (windowStart >= windowEnd) windowStart = windowEnd - 1; // Ensure valid window


    // Draw Temperature's compressed graph and clear the area as it draws.
    drawCompressedGraph(dataManager, true, true, windowStart, windowEnd);

    // Draw the humidity's compressed graph on top, without clearing.
    drawCompressedGraph(dataManager, false, false, windowStart, windowEnd);

    // --- Hybrid Clearing: Clear the raw data area ---
    // The compressed data ends where the raw data begins.
    uint32_t raw_hist_duration = latest_raw_timestamp - raw_timestamps[0];
    uint32_t compressed_end_time = latest_raw_timestamp - raw_hist_duration;
    int raw_start_x = mapFloat(compressed_end_time, windowStart, windowEnd, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
    if (raw_start_x >= PLOT_X_START && raw_start_x < PLOT_X_START + PLOT_WIDTH) {
         tft.fillRect(raw_start_x, PLOT_Y_START, (PLOT_X_START + PLOT_WIDTH) - raw_start_x, PLOT_HEIGHT, TFT_BLACK);
    }

    // Overlay both raw graphs on top of the compressed graphs.
    drawRawGraph(dataManager, true, windowStart, windowEnd);
    drawRawGraph(dataManager, false, windowStart, windowEnd);

    // Draw axis labels last so they are not overwritten.
    drawGridAndAxes(dataManager);

    // Redraw border on top
    tft.drawRect(PLOT_X_START - 1, PLOT_Y_START - 1, PLOT_WIDTH + 2, PLOT_HEIGHT + 2, TFT_WHITE);
}

void GraphRenderer::drawGridAndAxes(const GraphDataManager* dataManager) {
    // This function is now responsible only for drawing the Y-axis labels.
    // The dynamic grid/boundary lines are drawn in drawCompressedGraph.
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

void GraphRenderer::drawRawGraph(const GraphDataManager* dataManager, bool isTemp, uint32_t windowStart, uint32_t windowEnd) {
    const uint16_t count = dataManager->getRawDataCount();
    if (count < 2) return;

    const float* data = isTemp ? dataManager->getRawTempHistory() : dataManager->getRawHumidityHistory();
    const uint32_t* timestamps = dataManager->getRawTimestamps();
    const uint16_t color = isTemp ? TEMP_COLOR : HUMIDITY_COLOR;
    const float y_min = dataManager->getMinValue(isTemp);
    const float y_max = dataManager->getMaxValue(isTemp);

    for (uint16_t i = 0; i < count - 1; ++i) {
        if (!isnan(data[i]) && !isnan(data[i+1])) {
            // Only draw lines that are at least partially within the window
            if (timestamps[i+1] >= windowStart && timestamps[i] <= windowEnd) {
                int16_t x1 = mapFloat(timestamps[i], windowStart, windowEnd, PLOT_X_START, PLOT_X_START + PLOT_WIDTH -1);
                int16_t y1 = mapFloat(data[i], y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);
                int16_t x2 = mapFloat(timestamps[i+1], windowStart, windowEnd, PLOT_X_START, PLOT_X_START + PLOT_WIDTH -1);
                int16_t y2 = mapFloat(data[i+1], y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);
                tft.drawLine(x1, y1, x2, y2, color);
            }
        }
    }
}


void GraphRenderer::drawCompressedGraph(const GraphDataManager* dataManager, bool isTemp, bool clear_under, uint32_t windowStart, uint32_t windowEnd) {
    const uint8_t segmentCount = dataManager->getSegmentCount();
    if (segmentCount == 0) return;

    const PolynomialSegment* segments = isTemp ? dataManager->getTempSegments() : dataManager->getHumiditySegments();
    const uint16_t color = isTemp ? TEMP_COLOR : HUMIDITY_COLOR;
    const float y_min = dataManager->getMinValue(isTemp);
    const float y_max = dataManager->getMaxValue(isTemp);

    const uint32_t* raw_timestamps = dataManager->getRawTimestamps();
    const uint16_t raw_data_count = dataManager->getRawDataCount();
    if (raw_data_count < 1) return;

    // The compressed data ends where the raw data logging buffer begins.
    uint32_t raw_hist_duration = raw_timestamps[raw_data_count - 1] - raw_timestamps[0];
    uint32_t compressed_data_end_time = raw_timestamps[raw_data_count - 1] - raw_hist_duration;

    int16_t last_y = -1;

    // Draw marker for boundary between raw and compressed data
    int boundary_x = mapFloat(compressed_data_end_time, windowStart, windowEnd, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
    if (boundary_x >= PLOT_X_START && boundary_x < PLOT_X_START + PLOT_WIDTH) {
        tft.drawFastVLine(boundary_x, PLOT_Y_START, PLOT_HEIGHT, TFT_GREEN);
    }

    uint32_t t_cursor = compressed_data_end_time;
    int seg_idx = segmentCount - 1;
    int poly_idx = dataManager->getCurrentPolyIndex() -1;
    if (poly_idx < 0) {
      poly_idx = POLY_COUNT_PER_SEGMENT - 1;
      seg_idx--; // Start from the previous segment if the current one is empty
    }


    // Iterate backwards through time from the start of the raw data
    for (int x_screen = boundary_x; x_screen >= PLOT_X_START; --x_screen) {
        if (clear_under) {
            tft.drawFastVLine(x_screen, PLOT_Y_START, PLOT_HEIGHT, TFT_BLACK);
        }

        uint32_t t_target = mapFloat(x_screen, PLOT_X_START, PLOT_X_START + PLOT_WIDTH, windowStart, windowEnd);

        // Find the correct polynomial segment for this timestamp
        bool found_poly = false;
        while(seg_idx >= 0) {
             if (poly_idx < 0) { // Move to the previous segment
                 seg_idx--;
                 poly_idx = POLY_COUNT_PER_SEGMENT - 1;
                 continue;
             }
             uint32_t poly_delta = segments[seg_idx].timeDeltas[poly_idx];
             if (poly_delta == 0) { // Skip empty polys
                 poly_idx--;
                 continue;
             }

             if (t_target <= t_cursor && t_target >= t_cursor - poly_delta) {
                 found_poly = true;
                 break;
             }

             t_cursor -= poly_delta;
             // Draw boundary line between polynomials
             int poly_boundary_x = mapFloat(t_cursor, windowStart, windowEnd, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
             if (poly_boundary_x >= PLOT_X_START && poly_boundary_x < boundary_x) {
                 tft.drawFastVLine(poly_boundary_x, PLOT_Y_START, PLOT_HEIGHT, 0x0821); // Dark blue
             }

             poly_idx--;
        }

        if(found_poly) {
            uint32_t poly_delta = segments[seg_idx].timeDeltas[poly_idx];
            double t_norm = (poly_delta > 0) ? (double)(t_target - (t_cursor - poly_delta)) / poly_delta : 0;
            float y_val = evaluatePolynomial(segments[seg_idx].coefficients[poly_idx], POLY_DEGREE, t_norm);
            int16_t y_screen = mapFloat(y_val, y_min, y_max, PLOT_Y_START + PLOT_HEIGHT -1, PLOT_Y_START);

            if (last_y != -1) {
                // Check if x_screen is monotonically decreasing to prevent weird lines
                if (x_screen < (boundary_x -1))
                  tft.drawLine(x_screen, y_screen, x_screen + 1, last_y, color);
            } else {
                tft.drawPixel(x_screen, y_screen, color);
            }
            last_y = y_screen;
        } else {
            last_y = -1; // We are out of data for this part of the window
        }
    }
}
