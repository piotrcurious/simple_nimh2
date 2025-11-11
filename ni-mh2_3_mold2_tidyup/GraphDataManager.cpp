#include "GraphDataManager.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>

// Helper function to evaluate a polynomial, adapted from the source project
static double evaluatePolynomial(const float *coefficients, uint8_t degree, double tNorm) {
    double result = 0.0;
    double tPower = 1.0;
    for (int i = 0; i <= degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= tNorm;
    }
    return result;
}

GraphDataManager::GraphDataManager()
    : raw_data_count(0), raw_time_delta(0), log_buffer_count(0), last_timestamp(0),
      segment_count(0), current_poly_index(0),
      min_temp_value(INFINITY), max_temp_value(-INFINITY),
      min_humidity_value(INFINITY), max_humidity_value(-INFINITY) {}

void GraphDataManager::begin() {
    // Initialize raw data buffers with NAN
    for (int i = 0; i < MAX_RAW_DATA_POINTS; ++i) {
        raw_temp_history[i] = NAN;
        raw_humidity_history[i] = NAN;
        raw_timestamps[i] = 0;
    }
}

void GraphDataManager::shiftRawHistoryLeft(float* arr, size_t len) {
    if (len <= 1) return;
    std::memmove(arr, arr + 1, (len - 1) * sizeof(float));
    arr[len - 1] = NAN;
}

void GraphDataManager::logData(float tempData, float humidityData, uint32_t currentTimestamp) {
    // --- 1. Update the raw data buffer for immediate display ---
    if (raw_data_count >= MAX_RAW_DATA_POINTS) {
        // Shift all raw data arrays left to make space
        shiftRawHistoryLeft(raw_temp_history, MAX_RAW_DATA_POINTS);
        shiftRawHistoryLeft(raw_humidity_history, MAX_RAW_DATA_POINTS);
        if (MAX_RAW_DATA_POINTS > 1) {
            std::memmove(raw_timestamps, raw_timestamps + 1, (MAX_RAW_DATA_POINTS - 1) * sizeof(uint32_t));
        }
        raw_data_count = MAX_RAW_DATA_POINTS - 1;
    }

    raw_temp_history[raw_data_count] = tempData;
    raw_humidity_history[raw_data_count] = humidityData;
    raw_timestamps[raw_data_count] = currentTimestamp;
    raw_data_count++;
    updateMinMax();

    // --- 2. Log data into the compression buffer ---
    if (last_timestamp == 0) last_timestamp = currentTimestamp;
    uint32_t timeDelta = currentTimestamp - last_timestamp;
    last_timestamp = currentTimestamp;

    temp_log_buffer[log_buffer_count] = tempData;
    humidity_log_buffer[log_buffer_count] = humidityData;
    timestamp_log_buffer[log_buffer_count] = timeDelta;
    log_buffer_count++;
    raw_time_delta += timeDelta;

    // --- 3. If the compression buffer is full, process it ---
    if (log_buffer_count >= LOG_BUFFER_POINTS_PER_POLY) {
        processDataBuffer();
    }
}

void GraphDataManager::updateMinMax() {
    min_temp_value = INFINITY;
    max_temp_value = -INFINITY;
    min_humidity_value = INFINITY;
    max_humidity_value = -INFINITY;

    for (uint16_t i = 0; i < raw_data_count; ++i) {
        if (!isnan(raw_temp_history[i])) {
            min_temp_value = std::min(min_temp_value, raw_temp_history[i]);
            max_temp_value = std::max(max_temp_value, raw_temp_history[i]);
        }
        if (!isnan(raw_humidity_history[i])) {
            min_humidity_value = std::min(min_humidity_value, raw_humidity_history[i]);
            max_humidity_value = std::max(max_humidity_value, raw_humidity_history[i]);
        }
    }

    // Add padding for better visualization
    float temp_span = max_temp_value - min_temp_value;
    if (temp_span < 1.0f) temp_span = 1.0f;
    min_temp_value -= temp_span * 0.05f;
    max_temp_value += temp_span * 0.05f;

    float humidity_span = max_humidity_value - min_humidity_value;
    if (humidity_span < 1.0f) humidity_span = 1.0f;
    min_humidity_value -= humidity_span * 0.05f;
    max_humidity_value += humidity_span * 0.05f;
}

void GraphDataManager::processDataBuffer() {
    if (segment_count == 0) {
        segment_count = 1;
        current_poly_index = 0;
        // Initialize the first segment
        for(int i = 0; i < POLY_COUNT_PER_SEGMENT; ++i) {
            temp_segment_buffer[0].timeDeltas[i] = 0;
            humidity_segment_buffer[0].timeDeltas[i] = 0;
        }
    }

    // Compress temperature and humidity data into the current segment
    compressDataToSegment(temp_log_buffer, timestamp_log_buffer, log_buffer_count, temp_segment_buffer[segment_count - 1], current_poly_index);
    compressDataToSegment(humidity_log_buffer, timestamp_log_buffer, log_buffer_count, humidity_segment_buffer[segment_count - 1], current_poly_index);

    // Reset the raw time delta and log buffer
    raw_time_delta = 0;
    log_buffer_count = 0;

    Serial.print("Added polynomial ");
    Serial.print(current_poly_index);
    Serial.print(" to segment ");
    Serial.println(segment_count - 1);

    current_poly_index++;

    // Check if the current segment is full
    if (current_poly_index >= POLY_COUNT_PER_SEGMENT) {
        current_poly_index = 0;
        if (segment_count < SEGMENT_COUNT) {
            segment_count++; // Add a new segment
            Serial.print("Created new segment ");
            Serial.println(segment_count - 1);
            // Initialize the new segment
             for(int i = 0; i < POLY_COUNT_PER_SEGMENT; ++i) {
                temp_segment_buffer[segment_count - 1].timeDeltas[i] = 0;
                humidity_segment_buffer[segment_count - 1].timeDeltas[i] = 0;
            }
        } else {
            // Buffer is full, recompress the oldest segments
            recompressData();
        }
    }
}

void GraphDataManager::compressDataToSegment(const float* data, const uint32_t* timestamps, uint16_t dataSize, PolynomialSegment& segment, uint16_t polyIndex) {
    if (dataSize == 0) return;

    std::vector<double> x_normalized(dataSize);
    std::vector<float> y(dataSize);
    uint32_t totalTime = 0;
    for (uint16_t i = 0; i < dataSize; ++i) {
        totalTime += timestamps[i];
    }
    if (totalTime == 0) totalTime = 1; // Avoid division by zero

    uint32_t cumulativeTime = 0;
    for (uint16_t i = 0; i < dataSize; ++i) {
        cumulativeTime += timestamps[i];
        x_normalized[i] = static_cast<double>(cumulativeTime) / totalTime;
        y[i] = data[i];
    }

    std::vector<float> coeffs = fitter.fitPolynomialD_superpos5c(x_normalized, y, POLY_DEGREE);

    // Store the coefficients and total time delta for this polynomial
    for (int i = 0; i <= POLY_DEGREE; ++i) {
        segment.coefficients[polyIndex][i] = (i < coeffs.size()) ? coeffs[i] : 0.0f;
    }
    segment.timeDeltas[polyIndex] = totalTime;
}

void GraphDataManager::recompressData() {
    if (segment_count < 2) return;

    // We'll combine the two oldest segments (at index 0 and 1) into a new segment at index 0
    PolynomialSegment recompressed_temp, recompressed_humidity;

    combinePolynomials(temp_segment_buffer[0], temp_segment_buffer[1], recompressed_temp);
    combinePolynomials(humidity_segment_buffer[0], humidity_segment_buffer[1], recompressed_humidity);

    // Replace the oldest segment with the new recompressed one
    temp_segment_buffer[0] = recompressed_temp;
    humidity_segment_buffer[0] = recompressed_humidity;

    // Shift the remaining segments (if any) to the left
    for (uint8_t i = 1; i < segment_count - 1; ++i) {
        temp_segment_buffer[i] = temp_segment_buffer[i + 1];
        humidity_segment_buffer[i] = humidity_segment_buffer[i + 1];
    }

    // The total number of segments is now one less.
    segment_count--;

    // Clear the last segment's data (which is now a duplicate)
    for (int i = 0; i < POLY_COUNT_PER_SEGMENT; ++i) {
        temp_segment_buffer[segment_count].timeDeltas[i] = 0;
        humidity_segment_buffer[segment_count].timeDeltas[i] = 0;
    }
    Serial.print("Recompressed. New segment count: ");
    Serial.println(segment_count);
    Serial.print("Poly data size: ");
    Serial.print(sizeof(temp_segment_buffer) + sizeof(humidity_segment_buffer));
    Serial.print(" bytes. Raw data size: ");
    Serial.println(sizeof(raw_temp_history) + sizeof(raw_humidity_history) + sizeof(raw_timestamps));
}


void GraphDataManager::combinePolynomials(const PolynomialSegment& oldest, const PolynomialSegment& secondOldest, PolynomialSegment& recompressedSegment) {
    uint16_t newPolyIndex = 0;

    // Process the oldest segment
    for (uint16_t i = 0; i < POLY_COUNT_PER_SEGMENT && newPolyIndex < POLY_COUNT_PER_SEGMENT; i += 2) {
        if (i + 1 >= POLY_COUNT_PER_SEGMENT || oldest.timeDeltas[i] == 0 || oldest.timeDeltas[i+1] == 0) continue;

        std::vector<float> newCoeffs = fitter.composePolynomials(oldest.coefficients[i], oldest.timeDeltas[i], oldest.coefficients[i+1], oldest.timeDeltas[i+1], POLY_DEGREE);

        for (int j = 0; j <= POLY_DEGREE; ++j) {
            recompressedSegment.coefficients[newPolyIndex][j] = (j < newCoeffs.size()) ? newCoeffs[j] : 0.0f;
        }
        recompressedSegment.timeDeltas[newPolyIndex] = oldest.timeDeltas[i] + oldest.timeDeltas[i+1];
        newPolyIndex++;
    }

    // Process the second oldest segment
    for (uint16_t i = 0; i < POLY_COUNT_PER_SEGMENT && newPolyIndex < POLY_COUNT_PER_SEGMENT; i += 2) {
        if (i + 1 >= POLY_COUNT_PER_SEGMENT || secondOldest.timeDeltas[i] == 0 || secondOldest.timeDeltas[i+1] == 0) continue;

        std::vector<float> newCoeffs = fitter.composePolynomials(secondOldest.coefficients[i], secondOldest.timeDeltas[i], secondOldest.coefficients[i+1], secondOldest.timeDeltas[i+1], POLY_DEGREE);

        for (int j = 0; j <= POLY_DEGREE; ++j) {
            recompressedSegment.coefficients[newPolyIndex][j] = (j < newCoeffs.size()) ? newCoeffs[j] : 0.0f;
        }
        recompressedSegment.timeDeltas[newPolyIndex] = secondOldest.timeDeltas[i] + secondOldest.timeDeltas[i+1];
        newPolyIndex++;
    }

    // Clear any remaining slots in the recompressed segment
    for (uint16_t i = newPolyIndex; i < POLY_COUNT_PER_SEGMENT; ++i) {
        recompressedSegment.timeDeltas[i] = 0;
    }
}
