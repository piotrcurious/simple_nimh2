#include "home_screen.h"
#include "definitions.h"
#include <math.h>
#include <algorithm>
#include <cstring>   // memmove
#include <stdint.h>

// --- Mold configuration ---
constexpr float MOLD_HUMIDITY_THRESHOLD = 65.0f; // percent (change as needed)
constexpr int MOLD_SUBDIVISIONS = 4;             // subdivisions per basic grid cell
constexpr uint8_t DARK_YELLOW_R = 200;
constexpr uint8_t DARK_YELLOW_G = 150;
constexpr uint8_t DARK_YELLOW_B =   0;
constexpr uint8_t RED_R         = 255;
constexpr uint8_t RED_G         =   0;
constexpr uint8_t RED_B         =   0;


HomeScreen::HomeScreen()
    : lastRenderMs(0), lastGatherMs(0)
{
    // nothing else
}

void HomeScreen::begin() {
    // initialize with NaN to indicate "no data"
    for (size_t i = 0; i < SCREEN_WIDTH; ++i) {
        temp_history[i] = NAN;
        humidity_history[i] = NAN;
        dew_point_history[i] = NAN;
    }
}

inline float HomeScreen::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (!isValidSample(x)) return out_min;
    if (in_max == in_min) return (out_min + out_max) * 0.5f;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void HomeScreen::shiftHistoryLeft(float *arr, size_t len) noexcept {
    if (len <= 1) return;
    // shift left by one: use memmove for efficiency
    std::memmove(arr, arr + 1, (len - 1) * sizeof(float));
    arr[len - 1] = NAN; // new slot starts empty; caller will write the newest value
}

void HomeScreen::gatherData() {
    unsigned long now = millis();
    if (now - lastGatherMs < GATHER_INTERVAL_MS) return;
    lastGatherMs = now;

    // shift arrays left (drop oldest)
    shiftHistoryLeft(temp_history, SCREEN_WIDTH);
    shiftHistoryLeft(humidity_history, SCREEN_WIDTH);
    shiftHistoryLeft(dew_point_history, SCREEN_WIDTH);

    // read sensors (keep local temporaries)
    float t = sht4Sensor.getTemperature();
    float h = sht4Sensor.getHumidity();
    float d = static_cast<float>(calculateDewPoint(t, h));

    // append newest samples at the rightmost index
    temp_history[SCREEN_WIDTH - 1] = t;
    humidity_history[SCREEN_WIDTH - 1] = h;
    dew_point_history[SCREEN_WIDTH - 1] = d;
}

void HomeScreen::render() {
    unsigned long now = millis();
    if (now - lastRenderMs < RENDER_INTERVAL_MS) return;
    lastRenderMs = now;

    // clear plotting region and render
    tft.fillRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, PLOT_HEIGHT, TFT_BLACK);
    drawGraph();
    drawLabels();
}

//
// Helper color conversion & mix (RGB888 <-> RGB565)
//
static inline uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static inline void rgb565To888(uint16_t color565, uint8_t &r, uint8_t &g, uint8_t &b) {
    uint8_t r5 = (color565 >> 11) & 0x1F;
    uint8_t g6 = (color565 >> 5) & 0x3F;
    uint8_t b5 = color565 & 0x1F;
    r = (r5 << 3) | (r5 >> 2);
    g = (g6 << 2) | (g6 >> 4);
    b = (b5 << 3) | (b5 >> 2);
}

static inline uint16_t mix565(uint16_t a, uint16_t b, float t) {
    if (t <= 0.0f) return a;
    if (t >= 1.0f) return b;
    uint8_t ar, ag, ab, br, bg, bb;
    rgb565To888(a, ar, ag, ab);
    rgb565To888(b, br, bg, bb);
    uint8_t rr = static_cast<uint8_t>(ar + (br - ar) * t);
    uint8_t rg = static_cast<uint8_t>(ag + (bg - ag) * t);
    uint8_t rb = static_cast<uint8_t>(ab + (bb - ab) * t);
    return rgbTo565(rr, rg, rb);
}

//
// Interpolate (linearly) humidity at an arbitrary horizontal pixel x (0..PLOT_WIDTH-1).
// Returns NAN if both adjacent samples are invalid.
//
float HomeScreen::interpolatedHumidityAtPixel(const float *history, size_t len, int px, float &outHumidity) {
    if (len < 2) { outHumidity = NAN; return NAN; }
    float scaleX = (len > 1) ? static_cast<float>(PLOT_WIDTH) / (len - 1) : 1.0f;
    // pixel px corresponds to fractional data index:
    float idxf = static_cast<float>(px) / scaleX;
    if (idxf <= 0.0f) {
        outHumidity = history[0];
        return outHumidity;
    }
    if (idxf >= static_cast<float>(len - 1)) {
        outHumidity = history[len - 1];
        return outHumidity;
    }
    int i = static_cast<int>(floorf(idxf));
    int j = i + 1;
    float frac = idxf - static_cast<float>(i);
    float hi = history[i];
    float hj = history[j];
    bool ok_i = isValidSample(hi);
    bool ok_j = isValidSample(hj);
    if (ok_i && ok_j) {
        outHumidity = hi + (hj - hi) * frac;
        return outHumidity;
    } else if (ok_i && !ok_j) {
        outHumidity = hi;
        return outHumidity;
    } else if (!ok_i && ok_j) {
        outHumidity = hj;
        return outHumidity;
    } else {
        outHumidity = NAN;
        return NAN;
    }
}

//
// Draw base grid (unchanged) and then draw mold grid only under humidity parts > threshold.
// drawGrid now accepts humidity history + its autoscale range.
//
void HomeScreen::drawGrid(const float *humidity_history, size_t history_len, float h_plot_min, float h_plot_max) {
    // --- base horizontal subtle grid lines ---
    tft.setTextSize(1);
    for (int g = 0; g <= GRID_LINES; ++g) {
        int gy = PLOT_Y_START + ((PLOT_HEIGHT * g) / GRID_LINES);
        tft.drawFastHLine(PLOT_X_START, gy, PLOT_WIDTH, TFT_DARKGREY);
    }

    // vertical dashed grid aligned to the right edge
    for (int gx = PLOT_X_START + PLOT_WIDTH; gx >= PLOT_X_START; gx -= TIME_GRID_STEP_PX) {
        int y = PLOT_Y_START;
        while (y <= PLOT_Y_START + PLOT_HEIGHT) {
            int drawLen = std::min(DASH_LEN_PX, (PLOT_Y_START + PLOT_HEIGHT) - y + 1);
            tft.drawFastVLine(gx, y, drawLen, TFT_DARKGREY);
            y += DASH_LEN_PX + GAP_LEN_PX;
        }
    }

    // --- mold-risk overlay under humidity curve segments ---
    // validate humidity axis
    if (!isfinite(h_plot_min) || !isfinite(h_plot_max) || fabs(h_plot_max - h_plot_min) < 1e-6f) {
        return;
    }

    // If threshold >= plotted max, nothing is above threshold.
    if (MOLD_HUMIDITY_THRESHOLD >= h_plot_max) return;

    // If threshold <= plotted min, entire vertical range is candidate (mask will be all true where humidity valid).
    // Map threshold to pixel Y
    float thresholdYf = mapFloat(MOLD_HUMIDITY_THRESHOLD, h_plot_min, h_plot_max,
                                static_cast<float>(PLOT_Y_START + PLOT_HEIGHT),
                                static_cast<float>(PLOT_Y_START));
    if (thresholdYf < static_cast<float>(PLOT_Y_START)) thresholdYf = static_cast<float>(PLOT_Y_START);
    if (thresholdYf > static_cast<float>(PLOT_Y_START + PLOT_HEIGHT)) thresholdYf = static_cast<float>(PLOT_Y_START + PLOT_HEIGHT);

    // compute mold lattice spacing aligned to base grid
    const float basic_spacing = static_cast<float>(PLOT_HEIGHT) / static_cast<float>(GRID_LINES);
    const float mold_step = basic_spacing / static_cast<float>(MOLD_SUBDIVISIONS);

    // Align start on mold lattice (anchor to PLOT_Y_START so alignment consistent)
    float relThreshold = thresholdYf - static_cast<float>(PLOT_Y_START); // distance from top in px
    float rem = fmodf(relThreshold, mold_step);
    if (rem < 0.0f) rem += mold_step;
    float startYf = thresholdYf - rem;
    if (startYf < static_cast<float>(PLOT_Y_START)) startYf = static_cast<float>(PLOT_Y_START);

    // build boolean mask across plot width: true if humidity (interpolated) > threshold
    // index 0..PLOT_WIDTH-1 correspond to x positions from left of plotting area
    std::vector<uint8_t> mask; // small memory, uint8_t as boolean
    mask.resize(PLOT_WIDTH);
    float scaleX = (history_len > 1) ? static_cast<float>(PLOT_WIDTH) / (history_len - 1) : 1.0f;
    for (int px = 0; px < PLOT_WIDTH; ++px) {
        float hum;
        interpolatedHumidityAtPixel(humidity_history, history_len, px, hum);
        if (!isValidSample(hum)) {
            mask[px] = 0;
        } else {
            mask[px] = (hum > MOLD_HUMIDITY_THRESHOLD) ? 1 : 0;
        }
    }

    // Precompute color endpoints
    uint16_t colorDarkYellow = rgbTo565(DARK_YELLOW_R, DARK_YELLOW_G, DARK_YELLOW_B);
    uint16_t colorRed       = rgbTo565(RED_R, RED_G, RED_B);

    float topYf = static_cast<float>(PLOT_Y_START);
    float denom = (thresholdYf - topYf);
    if (denom < 1e-6f) denom = 1.0f; // avoid div zero (degenerate case handled earlier)

    // Iterate mold horizontal lines from startYf up to top
    for (float y = startYf; y >= topYf - 0.5f; y -= mold_step) {
        // compute color mix t (0 at threshold, 1 at top)
        float t = (thresholdYf - y) / denom;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        uint16_t lineColor = mix565(colorDarkYellow, colorRed, t);

        // draw horizontal segments only where mask is true
        int runStart = -1;
        for (int px = 0; px < PLOT_WIDTH; ++px) {
            if (mask[px]) {
                if (runStart < 0) runStart = px;
            } else {
                if (runStart >= 0) {
                    // draw run from runStart .. px-1
                    int drawX = PLOT_X_START + runStart;
                    int drawLen = px - runStart;
                    tft.drawFastHLine(drawX, static_cast<int>(roundf(y)), drawLen, lineColor);
                    runStart = -1;
                }
            }
        }
        // final run if mask ended true at boundary
        if (runStart >= 0) {
            int drawX = PLOT_X_START + runStart;
            int drawLen = PLOT_WIDTH - runStart;
            tft.drawFastHLine(drawX, static_cast<int>(roundf(y)), drawLen, lineColor);
        }
    }

    // draw threshold delimiter (optional & visible) along the runs (only where mask true at that pixel)
    // we draw a bright thin line across only where mask true
    int thrY = static_cast<int>(roundf(thresholdYf));
    int runStart = -1;
    for (int px = 0; px < PLOT_WIDTH; ++px) {
        if (mask[px]) {
            if (runStart < 0) runStart = px;
        } else {
            if (runStart >= 0) {
                tft.drawFastHLine(PLOT_X_START + runStart, thrY, px - runStart, rgbTo565(255,200,0));
                runStart = -1;
            }
        }
    }
    if (runStart >= 0) {
        tft.drawFastHLine(PLOT_X_START + runStart, thrY, PLOT_WIDTH - runStart, rgbTo565(255,200,0));
    }
}


void HomeScreen::drawSeries(const float *history, size_t len,
                            float plot_min, float plot_max,
                            uint16_t color, bool /*rightAxis*/) {
    if (len < 2) return;

    // scale for x (in case SCREEN_WIDTH != PLOT_WIDTH)
    float scaleX = (len > 1) ? static_cast<float>(PLOT_WIDTH) / (len - 1) : 1.0f;

    for (size_t i = 0; i < len - 1; ++i) {
        int x1 = PLOT_X_START + static_cast<int>(round(i * scaleX));
        int x2 = PLOT_X_START + static_cast<int>(round((i + 1) * scaleX));

        float v1 = history[i];
        float v2 = history[i + 1];
        bool ok1 = isValidSample(v1);
        bool ok2 = isValidSample(v2);

        if (ok1 && ok2) {
            int y1 = static_cast<int>(mapFloat(v1, plot_min, plot_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START));
            int y2 = static_cast<int>(mapFloat(v2, plot_min, plot_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START));
            tft.drawLine(x1, y1, x2, y2, color);
        } else if (ok1 && !ok2) {
            int y1 = static_cast<int>(mapFloat(v1, plot_min, plot_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START));
            tft.drawPixel(x1, y1, color);
        } else if (!ok1 && ok2) {
            int y2 = static_cast<int>(mapFloat(v2, plot_min, plot_max, PLOT_Y_START + PLOT_HEIGHT, PLOT_Y_START));
            tft.drawPixel(x2, y2, color);
        }
    }
}

void HomeScreen::drawAxisLabels(float td_plot_min, float td_plot_max,
                               float h_plot_min, float h_plot_max) {
    // left side: temperature/dew point limits (shared)
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    String topTD = String(td_plot_max, 1) + "C";
    String botTD = String(td_plot_min, 1) + "C";
    tft.drawString(topTD, PLOT_X_START, PLOT_Y_START + 1);
    tft.drawString(botTD, PLOT_X_START, PLOT_Y_START + PLOT_HEIGHT - 10);

    // right side: humidity limits
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    String topH = String(h_plot_max, 0) + "%";
    String botH = String(h_plot_min, 0) + "%";
    tft.drawString(topH, PLOT_X_START + PLOT_WIDTH, PLOT_Y_START + 1);
    tft.drawString(botH, PLOT_X_START + PLOT_WIDTH, PLOT_Y_START + PLOT_HEIGHT - 10);
}


//
// Updated drawGraph() to pass humidity data into drawGrid.
//
void HomeScreen::drawGraph() {
    // --- autoscale temperature + dew point (shared) ---
    float td_min =  INFINITY, td_max = -INFINITY;
    for (size_t i = 0; i < SCREEN_WIDTH; ++i) {
        float t = temp_history[i];
        float d = dew_point_history[i];
        if (isValidSample(t)) {
            td_min = std::min(td_min, t);
            td_max = std::max(td_max, t);
        }
        if (isValidSample(d)) {
            td_min = std::min(td_min, d);
            td_max = std::max(td_max, d);
        }
    }
    if (td_min > td_max) { td_min = 0.0f; td_max = 50.0f; }

    float td_span = td_max - td_min;
    if (td_span < 0.001f) td_span = 1.0f;
    float td_pad = td_span * 0.05f;
    float td_plot_min = td_min - td_pad;
    float td_plot_max = td_max + td_pad;

    // --- autoscale humidity ---
    float h_min =  INFINITY, h_max = -INFINITY;
    for (size_t i = 0; i < SCREEN_WIDTH; ++i) {
        float h = humidity_history[i];
        if (isValidSample(h)) {
            h_min = std::min(h_min, h);
            h_max = std::max(h_max, h);
        }
    }
    if (h_min > h_max) { h_min = 0.0f; h_max = 100.0f; }
    float h_span = h_max - h_min;
    if (h_span < 0.001f) h_span = 1.0f;
    float h_pad = h_span * 0.05f;
    float h_plot_min = h_min - h_pad;
    float h_plot_max = h_max + h_pad;

    // draw grids first (pass humidity history so the mold overlay can clip horizontally)
    drawGrid(humidity_history, SCREEN_WIDTH, h_plot_min, h_plot_max);

    // draw series: temperature (red) and dew point (green) share scale
    drawSeries(temp_history, SCREEN_WIDTH, td_plot_min, td_plot_max, TFT_RED);
    drawSeries(dew_point_history, SCREEN_WIDTH, td_plot_min, td_plot_max, TFT_GREEN);

    // draw humidity on same plot area but using its own right-side axis
    drawSeries(humidity_history, SCREEN_WIDTH, h_plot_min, h_plot_max, TFT_BLUE, /*rightAxis=*/true);

    // border
    tft.drawRect(PLOT_X_START - 1, PLOT_Y_START - 1, PLOT_WIDTH + 2, PLOT_HEIGHT + 2, TFT_WHITE);

    // axis labels and (optional) legend
    drawAxisLabels(td_plot_min, td_plot_max, h_plot_min, h_plot_max);
}

void HomeScreen::drawLabels() {
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // current temperature (left)
    float tcur = temp_history[SCREEN_WIDTH - 1];
    tft.setTextColor(TFT_RED, TFT_BLACK);
    if (!isValidSample(tcur)) {
        tft.drawString("-- C", PLOT_X_START, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(tcur, 2) + " C", PLOT_X_START, PLOT_Y_START + 12);
    }

    // current humidity (to the right)
    float hcur = humidity_history[SCREEN_WIDTH - 1];
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    if (!isValidSample(hcur)) {
        tft.drawString("-- %", PLOT_X_START + 90, PLOT_Y_START + 12);
    } else {
        tft.drawString(String(hcur, 2) + " %", PLOT_X_START + 90, PLOT_Y_START + 12);
    }

    // dew point (smaller, under temperature)
    float dcur = dew_point_history[SCREEN_WIDTH - 1];
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    if (!isValidSample(dcur)) {
        tft.drawString("Dew: -- C", PLOT_X_START, PLOT_Y_START + 30);
    } else {
        tft.drawString("Dew: " + String(dcur, 2) + " C", PLOT_X_START, PLOT_Y_START + 30);
    }
}

double HomeScreen::calculateDewPoint(double temperature, double humidity) {
    // Magnus-Tetens formula (valid for typical temperature/humidity ranges)
    constexpr double a = 17.27;
    constexpr double b = 237.7;
    // protect against invalid humidity input
    double h = humidity;
    if (!isfinite(h) || h <= 0.0) h = 0.0001;    // avoid log(0) / division issues
    if (h > 100.0) h = 100.0;
    double alpha = ((a * temperature) / (b + temperature)) + log(h / 100.0);
    double dew_point = (b * alpha) / (a - alpha);
    return dew_point;
}
