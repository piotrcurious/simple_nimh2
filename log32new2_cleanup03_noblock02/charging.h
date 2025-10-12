#ifndef CHARGING_H
#define CHARGING_H

#include "definitions.h"

void startCharging();
void stopCharging();
void handleBatteryCharging();
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
void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay = STABILIZATION_DELAY_MS, unsigned long unloaded_delay = UNLOADED_VOLTAGE_DELAY_MS);
bool measurementStep();
bool fetchMeasurementResult(MHElectrodeData &out);
void abortMeasurement();
void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval);
bool findOptimalChargingDutyCycleStepAsync();

#endif // CHARGING_H