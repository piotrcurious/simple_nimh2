#include "logging.h"
#include "Shared.h" // For ChargeLogData struct

// Define the global chargeLog vector here
std::vector<ChargeLogData> chargeLog;

void logChargeData(const ChargeLogData& data) {
    chargeLog.push_back(data);
}
