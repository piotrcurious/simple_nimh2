#ifndef CHARGING_H
#define CHARGING_H

#include "definitions.h"

// Structured Thermal Replay History
struct ThermalStepResponse {
    uint32_t timestamp;
    float current;
    float voltage;
    float ambientTemp;
    float actualTemp;
    float predictedTemp;
    float predictedVoltage;
    float overpotential;
    float ir;
    float p_residual;
    float r_p;
};

constexpr size_t MAX_THERMAL_HISTORY = 60;

struct ThermalHistoryBuffer {
    ThermalStepResponse items[MAX_THERMAL_HISTORY];
    size_t count = 0;
    size_t head = 0; // index of oldest item if full, or next write index if not full

    void clear() { count = 0; head = 0; }
    void push_back(const ThermalStepResponse& item) {
        if (count < MAX_THERMAL_HISTORY) {
            items[count++] = item;
        } else {
            items[head] = item;
            head = (head + 1) % MAX_THERMAL_HISTORY;
        }
    }
    bool empty() const { return count == 0; }
    size_t size() const { return count; }
    const ThermalStepResponse& operator[](size_t idx) const {
        if (count < MAX_THERMAL_HISTORY) return items[idx];
        return items[(head + idx) % MAX_THERMAL_HISTORY];
    }
    ThermalStepResponse& operator[](size_t idx) {
        if (count < MAX_THERMAL_HISTORY) return items[idx];
        return items[(head + idx) % MAX_THERMAL_HISTORY];
    }
};

extern ThermalHistoryBuffer s_thermalHistory;
extern double prev_t1;
extern double prev_t2;
extern double t1_deriv;
extern double t2_deriv;
extern float predictedTempTrack;
extern unsigned long pulseCycleStartTime;
extern unsigned long lastLogTime;

// =======================================================
// Existing public charging interface
// =======================================================
void startCharging();
void stopCharging();
bool chargeBattery();
void updateDynamicMaximumCurrent();

MHElectrodeData measureMHElectrodeVoltage(int testDutyCycle);
int findOptimalChargingDutyCycle(int maxChargeDutyCycle, int suggestedStartDutyCycle);

float estimateTempDiff(
  float voltageUnderLoad,
  float voltageNoLoad,
  float current,
  float internalResistanceParam,
  float ambientTempC,
  uint32_t currentTime,
  uint32_t lastChargeEvaluationTime,
  float BatteryTempC,
  float* unappliedEnergy_J,
  float cellMassKg,
  float specificHeat,
  float area,
  float convectiveH,
  float emissivity
);

void startMHElectrodeMeasurement(
  int testDutyCycle,
  unsigned long stabilization_delay = STABILIZATION_DELAY_MS,
  unsigned long unloaded_delay = UNLOADED_VOLTAGE_DELAY_MS
);

bool measurementStep();
bool fetchMeasurementResult(MHElectrodeData &out);
void abortMeasurement();

void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval);
bool findOptimalChargingDutyCycleStepAsync();


// =======================================================
// --- New: Absolute Temperature Estimation API ---
// =======================================================

/**
 * @brief Pushes a new ChargeLogData entry into the short-term replay buffer
 *        used for absolute temperature estimation.
 * @param entry ChargeLogData structure containing timestamp, current, voltage,
 *              ambientTemperature, batteryTemperature, and resistance data.
 */
void pushRecentChargeLog(const ChargeLogData &entry);

/**
 * @brief Compute the absolute temperature rise by replaying the last N logged
 *        power/time entries via estimateTempDiff().
 * @param depth Number of historical entries to replay (will be clamped to available history).
 * @return Temperature rise (°C) relative to the batteryTemperature of the oldest entry.
 */
float computeAbsoluteTempRiseFromHistory(int depth);

/**
 * @brief Set the number of log entries used for absolute temperature estimation.
 * @param d Desired depth (1..TEMPRISE_ABS_MAX_DEPTH)
 */
void setTempriseAbsDepth(int d);

/**
 * @brief Set the blending factor between relative and absolute temperature estimation.
 *        final = balance * relative + (1 - balance) * absolute
 * @param b Blending factor (0.0–1.0)
 */
void setTempriseBalance(float b);

/**
 * @brief Get the current blending factor.
 */
float getTempriseBalance();

/**
 * @brief Get the current replay depth.
 */
int getTempriseAbsDepth();

int indexOfOldestEntry();

#endif // CHARGING_H
