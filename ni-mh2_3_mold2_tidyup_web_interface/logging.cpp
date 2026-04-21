#include "logging.h"

// Define the global chargeLog vector here
#ifndef MOCK_TEST
std::vector<ChargeLogData> chargeLog;
#endif

void logChargeData(const ChargeLogData& data) {
    chargeLog.push_back(data);
}
