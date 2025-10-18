#ifndef CHARGER_H
#define CHARGER_H

#include "definitions.h"
#include <vector>
#include <Eigen/Dense>

class DataPlotter; // Forward declaration

class Charger {
public:
    Charger(int pwmPin, DataPlotter& plotter);
    void begin();
    void update();
    void startCharging();
    void startModelBuilding();
    bool isModelBuilt() const;
    uint32_t getDutyCycle() const;
    float estimateCurrent(int dutyCycle);

private:
    void setupPWM();
    void buildCurrentModelStep();
    bool chargeBattery();
    void processThermistorData(const MeasurementData& data, const String& measurementType);
    void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);


    int _pwmPin;
    DataPlotter& _plotter;
    CurrentModel _currentModel;
    uint32_t _dutyCycle;

    // State for buildCurrentModelStep
    int _buildModelStep;
    int _buildModelDutyCycle;
    unsigned long _buildModelLastStepTime;
    std::vector<float> _dutyCycles;
    std::vector<float> _currents;

    // Charging state
    unsigned long _lastChargingHouseTime;
};

#endif // CHARGER_H