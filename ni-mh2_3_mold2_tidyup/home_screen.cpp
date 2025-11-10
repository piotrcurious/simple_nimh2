#include "home_screen.h"
#include "definitions.h"
#include <algorithm>

// Local helper function for polynomial fitting, adapted from the example sketch
namespace {
    void compressDataToSegment(
        const PolynomialSegment* segments,
        uint8_t count,
        uint16_t polyindex,
        const float* rawData,
        const uint32_t* timestamps,
        uint16_t dataSize,
        float* coefficients, // Output
        uint32_t& timeDelta) // Output
    {
        AdvancedPolynomialFitter fitter;
        int8_t segmentIndex = count - 1;
        int16_t polyIndex = polyindex;

        float timestamp_absolute = 0;
        for (uint16_t j = 0; j < dataSize; j++) {
            timestamp_absolute += timestamps[j];
        }

        std::vector<float> y(rawData, rawData + dataSize);

        // Define a lambda function to generate normalized time values on the fly
        auto x_func = [&](int i) {
            float current_time = 0;
            for (int j = 0; j <= i; ++j) {
                current_time += timestamps[j];
            }
            return HomeScreen::normalizeTime(current_time, timestamp_absolute);
        };

        std::vector<float> fitted_coefficients = fitter.fitPolynomialD_superpos5c(y, x_func, dataSize, POLY_DEGREE, AdvancedPolynomialFitter::NONE);

        for (uint8_t j = 0; j < fitted_coefficients.size() && j < POLY_DEGREE + 1; j++) {
            coefficients[j] = fitted_coefficients[j];
        }
        timeDelta = timestamp_absolute;
    }
}


HomeScreen::HomeScreen()
    : lastRenderMs(0), lastGatherMs(0)
{
    // Constructor
}

void HomeScreen::begin() {
    // Initialize any necessary variables, if needed.
    // The polynomial buffers are zero-initialized by default.
    lastTimestamp = millis() / 1000;
}

void HomeScreen::gatherData() {
    unsigned long now = millis();
    if (now - lastGatherMs < GATHER_INTERVAL_MS) return;
    lastGatherMs = now;

    float t = sht4Sensor.getTemperature();
    float h = sht4Sensor.getHumidity();

    if (isValidSample(t) && isValidSample(h)) {
        logSensorData(t, h);
    }
}

void HomeScreen::logSensorData(float temp, float humidity) {
    uint32_t currentTimestamp = millis() / 1000;
    uint32_t timeDelta = (currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;

    // Store in polynomial log buffer
    temp_log_buffer[log_buffer_index] = temp;
    humidity_log_buffer[log_buffer_index] = humidity;
    timestamp_log_buffer[log_buffer_index] = timeDelta;
    log_buffer_index++;

    // Store in raw data circular buffer
    raw_data_buffer[raw_data_head] = {currentTimestamp, temp, humidity};
    raw_data_head = (raw_data_head + 1) % RAW_DATA_BUFFER_SIZE;

    if (log_buffer_index >= LOG_BUFFER_POINTS_PER_POLY) {
        fitAndStorePolynomials();
        log_buffer_index = 0; // Reset buffer
    }
}

void HomeScreen::fitAndStorePolynomials() {
    if (segment_count == 0) {
        segment_count = 1;
        current_poly_index = 0;
        // Initialize timeDeltas to 0
        for (auto& seg : temp_segment_buffer) {
            for (auto& delta : seg.timeDeltas) delta = 0;
        }
        for (auto& seg : humidity_segment_buffer) {
            for (auto& delta : seg.timeDeltas) delta = 0;
        }
    }

    if (current_poly_index >= POLY_COUNT) {
        if (segment_count < SEGMENTS) {
            tail = (tail + 1) % SEGMENTS;
            segment_count++;
        } else {
            recompressSegments();
        }
        current_poly_index = 0;
    }

    uint32_t temp_time_delta, humidity_time_delta;

    // Fit Temperature Data
    compressDataToSegment(
        temp_segment_buffer, segment_count, current_poly_index,
        temp_log_buffer, timestamp_log_buffer, log_buffer_index,
        temp_segment_buffer[tail].coefficients[current_poly_index],
        temp_time_delta
    );
    temp_segment_buffer[tail].timeDeltas[current_poly_index] = temp_time_delta;

    // Fit Humidity Data
    compressDataToSegment(
        humidity_segment_buffer, segment_count, current_poly_index,
        humidity_log_buffer, timestamp_log_buffer, log_buffer_index,
        humidity_segment_buffer[tail].coefficients[current_poly_index],
        humidity_time_delta
    );
    humidity_segment_buffer[tail].timeDeltas[current_poly_index] = humidity_time_delta;

    current_poly_index++;
}

void HomeScreen::recompressSegments() {
    if (segment_count < 2) return;

    AdvancedPolynomialFitter fitter;

    // Recompress temperature segments
    PolynomialSegment recompressed_temp;
    uint16_t temp_poly_count = 0;
    for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
        if (temp_segment_buffer[head].timeDeltas[i] == 0 || temp_segment_buffer[head].timeDeltas[i+1] == 0) break;
        double combined_delta = temp_segment_buffer[head].timeDeltas[i] + temp_segment_buffer[head].timeDeltas[i+1];
        std::vector<float> new_coeffs = fitter.composePolynomials(temp_segment_buffer[head].coefficients[i], temp_segment_buffer[head].timeDeltas[i], temp_segment_buffer[head].coefficients[i+1], temp_segment_buffer[head].timeDeltas[i+1], POLY_DEGREE);
        for(uint8_t j = 0; j < new_coeffs.size() && j < POLY_DEGREE + 1; j++) {
            recompressed_temp.coefficients[temp_poly_count][j] = new_coeffs[j];
        }
        recompressed_temp.timeDeltas[temp_poly_count] = combined_delta;
        temp_poly_count++;
    }
     for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
        if (temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i] == 0 || temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1] == 0) break;
        double combined_delta = temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i] + temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1];
        std::vector<float> new_coeffs = fitter.composePolynomials(temp_segment_buffer[(head + 1) % SEGMENTS].coefficients[i], temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i], temp_segment_buffer[(head + 1) % SEGMENTS].coefficients[i+1], temp_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1], POLY_DEGREE);
        for(uint8_t j = 0; j < new_coeffs.size() && j < POLY_DEGREE + 1; j++) {
            recompressed_temp.coefficients[temp_poly_count][j] = new_coeffs[j];
        }
        recompressed_temp.timeDeltas[temp_poly_count] = combined_delta;
        temp_poly_count++;
    }

    // Recompress humidity segments
    PolynomialSegment recompressed_hum;
    uint16_t hum_poly_count = 0;
    for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
        if (humidity_segment_buffer[head].timeDeltas[i] == 0 || humidity_segment_buffer[head].timeDeltas[i+1] == 0) break;
        double combined_delta = humidity_segment_buffer[head].timeDeltas[i] + humidity_segment_buffer[head].timeDeltas[i+1];
        std::vector<float> new_coeffs = fitter.composePolynomials(humidity_segment_buffer[head].coefficients[i], humidity_segment_buffer[head].timeDeltas[i], humidity_segment_buffer[head].coefficients[i+1], humidity_segment_buffer[head].timeDeltas[i+1], POLY_DEGREE);
        for(uint8_t j = 0; j < new_coeffs.size() && j < POLY_DEGREE + 1; j++) {
            recompressed_hum.coefficients[hum_poly_count][j] = new_coeffs[j];
        }
        recompressed_hum.timeDeltas[hum_poly_count] = combined_delta;
        hum_poly_count++;
    }
    for (uint16_t i = 0; i < POLY_COUNT; i += 2) {
        if (humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i] == 0 || humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1] == 0) break;
        double combined_delta = humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i] + humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1];
        std::vector<float> new_coeffs = fitter.composePolynomials(humidity_segment_buffer[(head + 1) % SEGMENTS].coefficients[i], humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i], humidity_segment_buffer[(head + 1) % SEGMENTS].coefficients[i+1], humidity_segment_buffer[(head + 1) % SEGMENTS].timeDeltas[i+1], POLY_DEGREE);
        for(uint8_t j = 0; j < new_coeffs.size() && j < POLY_DEGREE + 1; j++) {
            recompressed_hum.coefficients[hum_poly_count][j] = new_coeffs[j];
        }
        recompressed_hum.timeDeltas[hum_poly_count] = combined_delta;
        hum_poly_count++;
    }

    // Replace oldest segment with recompressed data
    temp_segment_buffer[head] = recompressed_temp;
    humidity_segment_buffer[head] = recompressed_hum;

    head = (head + 1) % SEGMENTS;
    segment_count--;
}


void HomeScreen::render() {
    unsigned long now = millis();
    if (now - lastRenderMs < RENDER_INTERVAL_MS) return;
    lastRenderMs = now;

    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    renderPolynomialGraph();
    drawLabels();
}

void HomeScreen::drawLabels() {
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // Title
    tft.setCursor(PLOT_X_START, PLOT_Y_START - 10);
    tft.print("T/H 24h History");


    // current temperature (left)
    float tcur = temp_log_buffer[log_buffer_index > 0 ? log_buffer_index - 1 : 0];
    tft.setTextColor(TFT_RED, TFT_BLACK);
    if (!isValidSample(tcur)) {
        tft.drawString("-- C", PLOT_X_START, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(tcur, 2) + " C", PLOT_X_START, PLOT_Y_START + 12);
    }

    // current humidity (to the right)
    float hcur = humidity_log_buffer[log_buffer_index > 0 ? log_buffer_index - 1 : 0];
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    if (!isValidSample(hcur)) {
        tft.drawString("-- %", PLOT_X_START + 90, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(hcur, 2) + " %", PLOT_X_START + 90, PLOT_Y_START + 12);
    }

    // dew point (smaller, under temperature)
    float dcur = calculateDewPoint(tcur, hcur);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    if (!isValidSample(dcur)) {
        tft.drawString("Dew: -- C", PLOT_X_START, PLOT_Y_START + 30);
    } else {
        tft.drawString("Dew: " + String(dcur, 2) + " C", PLOT_X_START, PLOT_Y_START + 30);
    }
}

// Helper function to map a value from one range to another
static inline float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min == 0) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void HomeScreen::adjustTimeWindow(long hours) {
    long long newOffset = (long long)graphTimeOffset + (long long)hours * 3600LL;

    if (newOffset < 0) {
        newOffset = 0;
    }

    uint32_t current_time_seconds = millis() / 1000;
    if (newOffset > current_time_seconds) {
        newOffset = current_time_seconds;
    }

    graphTimeOffset = newOffset;
}


void HomeScreen::updateMinMax(const PolynomialSegment* segments, int seg_count, int poly_idx, float& min_val, float& max_val, uint32_t window_start, uint32_t window_end) {
    min_val = INFINITY;
    max_val = -INFINITY;
    if (seg_count == 0) return;

    int current_seg = seg_count - 1;
    int current_poly = poly_idx -1;
    uint32_t current_time_marker = millis() / 1000;

    while (current_seg >= 0) {
        if (current_poly < 0) {
            current_seg--;
            if (current_seg < 0) break;
            current_poly = POLY_COUNT - 1;
        }

        uint32_t delta = segments[current_seg].timeDeltas[current_poly];
        if (delta == 0) {
            current_poly--;
            continue;
        }

        uint32_t poly_end_time = current_time_marker;
        uint32_t poly_start_time = current_time_marker - delta;

        // Check if this polynomial is relevant to the window
        if (poly_start_time < window_end && poly_end_time > window_start) {
            const int steps = 20;
            for (int i = 0; i <= steps; ++i) {
                float t_norm = static_cast<float>(i) / steps;
                uint32_t time_in_poly = t_norm * delta;
                uint32_t actual_time = poly_start_time + time_in_poly;

                if (actual_time >= window_start && actual_time <= window_end) {
                    float val = evaluatePolynomial(segments[current_seg].coefficients[current_poly], POLY_DEGREE + 1, t_norm);
                    if (val < min_val) min_val = val;
                    if (val > max_val) max_val = val;
                }
            }
        }

        current_time_marker -= delta;
        if (current_time_marker < window_start) break;
        current_poly--;
    }
    if (isinf(min_val) || isinf(max_val)) {
        min_val = 0;
        max_val = 1;
    }
}

void HomeScreen::drawPolynomialSeries(const PolynomialSegment* segments, int seg_count, int poly_idx, uint32_t window_start, uint32_t window_end, float min_val, float max_val, uint16_t color) {
    if (seg_count == 0) return;

    // Draw boundary lines first
    uint32_t boundary_time = millis() / 1000;
    for (int s = seg_count - 1; s >= 0; --s) {
        for (int p = (s == seg_count - 1 ? poly_idx - 1 : POLY_COUNT - 1); p >= 0; --p) {
            uint32_t delta = segments[s].timeDeltas[p];
            if (delta == 0) continue;

            if (boundary_time >= window_start && boundary_time <= window_end) {
                int x = map_value(boundary_time, window_start, window_end, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
                tft.drawFastVLine(x, PLOT_Y_START, PLOT_HEIGHT, TFT_DARKGREY);
            }
            boundary_time -= delta;
            if (boundary_time < window_start) break;
        }
        if (boundary_time < window_start) break;
    }


    int16_t last_y = -1;

    int current_seg = seg_count - 1;
    int current_poly = poly_idx -1;
    uint32_t time_marker = millis() / 1000;

    for (int x = PLOT_WIDTH - 1; x >= 0; --x) {
        uint32_t target_time = window_start + (uint32_t)((float)x / PLOT_WIDTH * (window_end - window_start));

        // Find the correct polynomial for target_time
        bool poly_found = false;
        while(current_seg >= 0) {
             uint32_t delta = segments[current_seg].timeDeltas[current_poly];
             if (time_marker >= target_time && (time_marker - delta) <= target_time) {
                 poly_found = true;
                 break;
             }
             time_marker -= delta;
             current_poly--;
             if (current_poly < 0) {
                 current_seg--;
                 if(current_seg >= 0) current_poly = POLY_COUNT - 1;
             }
        }

        if (poly_found) {
            uint32_t delta = segments[current_seg].timeDeltas[current_poly];
            double time_in_poly = target_time - (time_marker - delta);
            double t_norm = normalizeTime(time_in_poly, delta);

            float val = evaluatePolynomial(segments[current_seg].coefficients[current_poly], POLY_DEGREE + 1, t_norm);
            int16_t y = map_value(val, min_val, max_val, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);

            if (y >= PLOT_Y_START && y < PLOT_Y_START + PLOT_HEIGHT) {
                 if (last_y != -1) {
                    tft.drawLine(x, y, x + 1, last_y, color);
                } else {
                    tft.drawPixel(x, y, color);
                }
            }
            last_y = y;
        } else {
            last_y = -1;
        }
    }
}


void HomeScreen::renderPolynomialGraph() {
    if (segment_count == 0) {
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 10);
        tft.println("No data yet...");
        return;
    }

    uint32_t now = millis() / 1000;
    uint32_t window_duration = 24 * 3600;
    uint32_t window_end = now - graphTimeOffset;
    uint32_t window_start = (window_end > window_duration) ? (window_end - window_duration) : 0;
    if (window_start > window_end) window_start = 0; // Rollover check

    float temp_min, temp_max, hum_min, hum_max;
    updateMinMax(temp_segment_buffer, segment_count, current_poly_index, temp_min, temp_max, window_start, window_end);
    updateMinMax(humidity_segment_buffer, segment_count, current_poly_index, hum_min, hum_max, window_start, window_end);

    // Add some padding to the min/max
    float temp_range = temp_max - temp_min;
    temp_max += temp_range * 0.1;
    temp_min -= temp_range * 0.1;

    float hum_range = hum_max - hum_min;
    hum_max += hum_range * 0.1;
    hum_min -= hum_range * 0.1;


    drawPolynomialSeries(temp_segment_buffer, segment_count, current_poly_index, window_start, window_end, temp_min, temp_max, TFT_RED);
    drawPolynomialSeries(humidity_segment_buffer, segment_count, current_poly_index, window_start, window_end, hum_min, hum_max, TFT_BLUE);
    drawRawDataOverlay(window_start, window_end, temp_min, temp_max, hum_min, hum_max);
    drawAxisLabels(temp_min, temp_max, hum_min, hum_max);
}

void HomeScreen::drawAxisLabels(float temp_min, float temp_max, float hum_min, float hum_max) {
    tft.setTextSize(1);

    // Temperature labels (left side)
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(String(temp_max, 1) + "C", PLOT_X_START + 2, PLOT_Y_START + 2, 1);
    tft.setTextDatum(BL_DATUM);
    tft.drawString(String(temp_min, 1) + "C", PLOT_X_START + 2, PLOT_Y_START + PLOT_HEIGHT - 2, 1);

    // Humidity labels (right side)
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String(hum_max, 0) + "%", PLOT_X_START + PLOT_WIDTH - 2, PLOT_Y_START + 2, 1);
    tft.setTextDatum(BR_DATUM);
    tft.drawString(String(hum_min, 0) + "%", PLOT_X_START + PLOT_WIDTH - 2, PLOT_Y_START + PLOT_HEIGHT - 2, 1);
}

void HomeScreen::drawRawDataOverlay(uint32_t window_start, uint32_t window_end, float temp_min, float temp_max, float hum_min, float hum_max) {
    for (int i = 0; i < RAW_DATA_BUFFER_SIZE; ++i) {
        const RawDataPoint& point = raw_data_buffer[i];
        if (point.timestamp >= window_start && point.timestamp <= window_end) {

            // Draw temperature point
            if (isValidSample(point.temperature)) {
                int x = map_value(point.timestamp, window_start, window_end, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
                int y = map_value(point.temperature, temp_min, temp_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
                if (x >= PLOT_X_START && x < PLOT_X_START + PLOT_WIDTH && y >= PLOT_Y_START && y < PLOT_Y_START + PLOT_HEIGHT) {
                    tft.drawPixel(x, y, TFT_WHITE);
                }
            }

            // Draw humidity point
            if (isValidSample(point.humidity)) {
                int x = map_value(point.timestamp, window_start, window_end, PLOT_X_START, PLOT_X_START + PLOT_WIDTH);
                int y = map_value(point.humidity, hum_min, hum_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START);
                if (x >= PLOT_X_START && x < PLOT_X_START + PLOT_WIDTH && y >= PLOT_Y_START && y < PLOT_Y_START + PLOT_HEIGHT) {
                    tft.drawPixel(x, y, TFT_CYAN);
                }
            }
        }
    }
}


double HomeScreen::calculateDewPoint(double temperature, double humidity) {
    constexpr double a = 17.27;
    constexpr double b = 237.7;
    double h = humidity;
    if (!isfinite(h) || h <= 0.0) h = 0.0001;
    if (h > 100.0) h = 100.0;
    double alpha = ((a * temperature) / (b + temperature)) + log(h / 100.0);
    double dew_point = (b * alpha) / (a - alpha);
    return dew_point;
}

// --- Static Helper Implementations ---
double HomeScreen::normalizeTime(double t, double tMax) {
    if (tMax == 0) return 0;
    return t / tMax;
}

float HomeScreen::evaluatePolynomial(const float* coefficients, uint8_t degree, double t) {
    double result = 0.0;
    double tPower = 1.0;
    for (int i = 0; i < degree; i++) {
        result += coefficients[i] * tPower;
        tPower *= t;
    }
    return static_cast<float>(result);
}
