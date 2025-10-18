#ifndef DISPLAY_UTILS_H
#define DISPLAY_UTILS_H

#include "Shared.h"

namespace DisplayUtils {

uint16_t darkerColor(uint16_t color, float darkeningFactor);
float mapf(float value, float in_min, float in_max, float out_min, float out_max);

} // namespace DisplayUtils

#endif // DISPLAY_UTILS_H