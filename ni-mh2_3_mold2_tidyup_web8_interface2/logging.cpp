#include "logging.h"
#include "definitions.h"

#ifndef MOCK_TEST
#define WEB_LOCK() if (webDataMutex) xSemaphoreTake(webDataMutex, portMAX_DELAY)
#define WEB_UNLOCK() if (webDataMutex) xSemaphoreGive(webDataMutex)
#else
#define WEB_LOCK()
#define WEB_UNLOCK()
#endif

// Define the global chargeLog vector here
std::vector<ChargeLogData> chargeLog;
const size_t MAX_CHARGE_LOG_SIZE = 500;

void logChargeData(const ChargeLogData& data) {
    WEB_LOCK();
    if (chargeLog.size() >= MAX_CHARGE_LOG_SIZE) {
        chargeLog.erase(chargeLog.begin());
    }
    chargeLog.push_back(data);
    WEB_UNLOCK();
}
