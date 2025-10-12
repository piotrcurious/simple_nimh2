#include "logging.h"

// Define the global chargeLog vector here
std::vector<ChargeLogData> chargeLog;

void logChargeData(const ChargeLogData& data) {
    chargeLog.push_back(data);
}