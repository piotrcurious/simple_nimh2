#ifndef CHARGING_H
#define CHARGING_H

#include "definitions.h"

// =======================================================
// Existing public charging interface
// =======================================================
void startCharging();
void stopCharging();
bool chargeBattery();

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
  float cellMassKg = DEFAULT_CELL_MASS_KG,
  float specificHeat = DEFAULT_SPECIFIC_HEAT,
  float area = DEFAULT_SURFACE_AREA_M2,
  float convectiveH = DEFAULT_CONVECTIVE_H,
  float emissivity = DEFAULT_EMISSIVITY
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
