#include "DisplayUtils.h"

namespace DisplayUtils {

// Improved function to get a darker and more gray shade of a color
uint16_t darkerColor(uint16_t color, float darkeningFactor) {
    // Ensure factor is within valid range
    darkeningFactor = max(0.0f, min(1.0f, darkeningFactor));
    float grayingFactor = 0.6f; // Adjust for desired level of grayness

    // Extract RGB components (5-6-5 format)
    uint8_t r = (color >> 11) & 0x1F;
    uint8_t g = (color >> 5) & 0x3F;
    uint8_t b = color & 0x1F;

    // Convert to floating point (0.0 to 1.0 range)
    float fr = r / 31.0f;
    float fg = g / 63.0f;
    float fb = b / 31.0f;

    // Calculate luminance
    float luminance = 0.2126 * fr + 0.7152 * fg + 0.0722 * fb;

    // Reduce saturation
    fr = fr * (1.0f - grayingFactor) + luminance * grayingFactor;
    fg = fg * (1.0f - grayingFactor) + luminance * grayingFactor;
    fb = fb * (1.0f - grayingFactor) + luminance * grayingFactor;

    // Apply Darkening
    fr *= (1.0f - darkeningFactor);
    fg *= (1.0f - darkeningFactor);
    fb *= (1.0f - darkeningFactor);

    // Clamp values
    fr = max(0.0f, min(1.0f, fr));
    fg = max(0.0f, min(1.0f, fg));
    fb = max(0.0f, min(1.0f, fb));

    // Convert back to 5-6-5 format
    uint16_t new_r = static_cast<uint16_t>(fr * 31.0f + 0.5f);
    uint16_t new_g = static_cast<uint16_t>(fg * 63.0f + 0.5f);
    uint16_t new_b = static_cast<uint16_t>(fb * 31.0f + 0.5f);

    return (new_r << 11) | (new_g << 5) | new_b;
}

// Float version of map function for better precision in scaling
float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

} // namespace DisplayUtils