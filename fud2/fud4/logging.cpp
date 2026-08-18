#include "logging.h"
#include "definitions.h"


// Define the global chargeLog vector here
std::vector<ChargeLogData> chargeLog;

void logChargeData(const ChargeLogData& data) {
    WEB_LOCK();
    if (chargeLog.capacity() < MAX_CHARGE_LOG_SIZE) {
        chargeLog.reserve(MAX_CHARGE_LOG_SIZE);
    }
    if (chargeLog.size() >= MAX_CHARGE_LOG_SIZE) {
        // Erase in batches of 100 to eliminate continuous memory shifting and heap fragmentation
        size_t eraseCount = (MAX_CHARGE_LOG_SIZE > 100) ? 100 : 1;
        if (chargeLog.size() >= eraseCount) {
            chargeLog.erase(chargeLog.begin(), chargeLog.begin() + eraseCount);
        } else {
            chargeLog.clear();
        }
    }
    chargeLog.push_back(data);
    WEB_UNLOCK();
}
