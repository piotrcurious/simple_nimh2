#ifndef POWER_H
#define POWER_H

#include "Shared.h"
#include "DataStore.h"

class Power {
public:
    Power();
    void begin();
    void setDutyCycle(uint32_t duty);
    void buildCurrentModel(DataStore& data);
    bool chargeBattery(DataStore& data);
    void startCharging(DataStore& data);
    void stopCharging(DataStore& data);

private:
    CurrentModel currentModel;
    int buildModelStep;
    int buildModelDutyCycle;
    unsigned long buildModelLastStepTime;
    std::vector<float> dutyCycles;
    std::vector<float> currents;

    ChargingState chargingState;
    uint32_t chargingStartTime;
    uint8_t overtemp_trip_counter;
    unsigned long lastChargeEvaluationTime;
    float maximumCurrent;
    float currentRampTarget;
    int lastOptimalDutyCycle;
};

#endif // POWER_H