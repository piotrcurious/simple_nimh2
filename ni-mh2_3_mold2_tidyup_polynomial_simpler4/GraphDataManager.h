#ifndef GRAPHDATAMANAGER_H
#define GRAPHDATAMANAGER_H

#include <vector>
#include <cstdint>
#include "AdvancedPolynomialFitter.hpp"
#include "definitions.h" // For SCREEN_WIDTH

// Constants adapted from the source project
constexpr int MAX_RAW_DATA_POINTS = 500; // Keep raw data buffer sized to the screen width for display
constexpr int LOG_BUFFER_POINTS_PER_POLY = 60;   // Number of points to accumulate before fitting a polynomial
constexpr int POLY_COUNT_PER_SEGMENT = 8;        // Number of polynomials in each segment
constexpr int SEGMENT_COUNT = 2;                 // Total number of segments to store
constexpr int POLY_DEGREE = 5;                   // Degree of the polynomial for fitting

// Data structure for a single polynomial segment
struct PolynomialSegment {
    float coefficients[POLY_COUNT_PER_SEGMENT][POLY_DEGREE + 1];
    uint32_t timeDeltas[POLY_COUNT_PER_SEGMENT];
};

class GraphDataManager {
public:
    GraphDataManager();

    void begin();
    void logData(float tempData, float humidityData, uint32_t currentTimestamp);

    // --- Public data accessors for the renderer ---
    // Raw data (for the live, scrolling part of the graph)
    const float* getRawTempHistory() const { return raw_temp_history; }
    const float* getRawHumidityHistory() const { return raw_humidity_history; }
    const uint32_t* getRawTimestamps() const { return raw_timestamps; }
    uint16_t getRawDataCount() const { return raw_data_count; }
    uint32_t getRawTimeDelta() const { return raw_time_delta; }
    float getMinValue(bool isTemp) const { return isTemp ? min_temp_value : min_humidity_value; }
    float getMaxValue(bool isTemp) const { return isTemp ? max_temp_value : max_humidity_value; }

    // Compressed data (for the historical, compacted part of the graph)
    const PolynomialSegment* getTempSegments() const { return temp_segment_buffer; }
    const PolynomialSegment* getHumiditySegments() const { return humidity_segment_buffer; }
    uint8_t getSegmentCount() const { return segment_count; }
    uint16_t getCurrentPolyIndex() const { return current_poly_index; }

private:
    void processDataBuffer();
    void compressDataToSegment(const float* data, const uint32_t* timestamps, uint16_t dataSize, PolynomialSegment& segment, uint16_t polyIndex);
    void recompressData();
    void combinePolynomials(const PolynomialSegment& oldest, const PolynomialSegment& secondOldest, PolynomialSegment& recompressedSegment);
    void shiftRawHistoryLeft(float* arr, size_t len);
    void updateMinMax();

    AdvancedPolynomialFitter fitter;

    // --- Raw data buffers for recent values ---
    float raw_temp_history[MAX_RAW_DATA_POINTS];
    float raw_humidity_history[MAX_RAW_DATA_POINTS];
    uint32_t raw_timestamps[MAX_RAW_DATA_POINTS];
    uint16_t raw_data_count;
    uint32_t raw_time_delta; // time since last compression, for graph alignment

    // --- Buffers for incoming data before compression ---
    float temp_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    float humidity_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    uint32_t timestamp_log_buffer[LOG_BUFFER_POINTS_PER_POLY];
    uint16_t log_buffer_count;
    uint32_t last_timestamp;

    // --- Storage for compressed polynomial segments ---
    PolynomialSegment temp_segment_buffer[SEGMENT_COUNT];
    PolynomialSegment humidity_segment_buffer[SEGMENT_COUNT];
    uint8_t segment_count;
    uint16_t current_poly_index;

    // --- Min/Max tracking for graph scaling ---
    float min_temp_value;
    float max_temp_value;
    float min_humidity_value;
    float max_humidity_value;
};

#endif // GRAPHDATAMANAGER_H
