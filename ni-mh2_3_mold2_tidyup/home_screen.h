#pragma once
#include <Arduino.h>
#include <math.h>
#include <vector>
#include <cstring>
#include "definitions.h"
#include "AdvancedPolynomialFitter.hpp"

// Constants for the polynomial graphing system
#define POLY_DEGREE 5
#define POLY_COUNT 8
#define SEGMENTS 2
#define LOG_BUFFER_POINTS_PER_POLY 60

// Storage structure for polynomial data
struct PolynomialSegment {
    float coefficients[POLY_COUNT][POLY_DEGREE + 1];
    uint32_t timeDeltas[POLY_COUNT];
};

// Structure for raw data points for overlay
struct RawDataPoint {
    uint32_t timestamp; // Changed to uint32_t for seconds
    float temperature;
    float humidity;
};

class HomeScreen {
public:
    HomeScreen();

    void begin();
    void render();
    void gatherData();
    void adjustTimeWindow(long hours);
    static double normalizeTime(double t, double tMax);

private:
    // timing (ms for render/gather, seconds for graphing)
    static constexpr unsigned long RENDER_INTERVAL_MS = 2000UL;
    static constexpr unsigned long GATHER_INTERVAL_MS = 60000UL;

    unsigned long lastRenderMs;
    unsigned long lastGatherMs;
    uint32_t lastTimestamp = 0; // Now in seconds

    // --- New Polynomial Graphing Members ---
    float temp_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    float humidity_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    uint32_t timestamp_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    uint16_t log_buffer_index = 0;

    PolynomialSegment temp_segment_buffer[SEGMENTS];
    PolynomialSegment humidity_segment_buffer[SEGMENTS];
    uint8_t segment_count = 0;
    uint16_t current_poly_index = 0;

    uint32_t graphTimeOffset = 0; // Now in seconds

    // --- Raw Data Buffer for Overlay ---
    static constexpr int RAW_DATA_BUFFER_SIZE = LOG_BUFFER_POINTS_PER_POLY * 2;
    RawDataPoint raw_data_buffer[RAW_DATA_BUFFER_SIZE];
    int raw_data_head = 0;

    // --- New Private Methods ---
    void logSensorData(float temp, float humidity);
    void fitAndStorePolynomials();
    void renderPolynomialGraph();
    void drawRawDataOverlay(uint32_t window_start, uint32_t window_end, float temp_min, float temp_max, float hum_min, float hum_max);
    void updateMinMax(const PolynomialSegment* segments, int seg_count, int poly_idx, float& min_val, float& max_val, uint32_t window_start, uint32_t window_end);
    void drawPolynomialSeries(const PolynomialSegment* segments, int seg_count, int poly_idx, uint32_t window_start, uint32_t window_end, float min_val, float max_val, uint16_t color);

    // Helpers
    void drawLabels();
    void drawAxisLabels(float temp_min, float temp_max, float hum_min, float hum_max);
    static inline bool isValidSample(float v) { return !isnan(v) && isfinite(v); }
    double calculateDewPoint(double temperature, double humidity);
    static float evaluatePolynomial(const float *coefficients, uint8_t degree, double t);
};
