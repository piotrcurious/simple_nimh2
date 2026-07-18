#include "logging.h"
#include "definitions.h"


// Define the global chargeLog vector here
std::vector<ChargeLogData> chargeLog;

void logChargeData(const ChargeLogData& data) {
    WEB_LOCK();
    if (chargeLog.size() >= MAX_CHARGE_LOG_SIZE) {
        chargeLog.erase(chargeLog.begin());
    }
    chargeLog.push_back(data);
    WEB_UNLOCK();
}
