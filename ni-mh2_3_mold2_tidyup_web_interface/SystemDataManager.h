#ifndef SYSTEM_DATA_MANAGER_H
#define SYSTEM_DATA_MANAGER_H

#ifndef SYSTEM_DATA_MANAGER_H_IMPL
#define SYSTEM_DATA_MANAGER_H_IMPL

#include <Arduino.h>
#include "adc_dma.h"
#include "SHT4xSensor.h"

struct SystemData {
    float battery_voltage_v;
    float charge_current_a;
    double ambient_temp_c;
    double battery_temp_c;
    double temp_diff_c;
    float vcc_mv;
    double mah_charged;
    uint32_t last_update_ms;
};

class SystemDataManager {
public:
    SystemDataManager(SHT4xSensor& sht4, int therm1Pin, int vccPin, double therm1Offset);

    void begin();
    void update(); // Should be called regularly (e.g., 20Hz)

    SystemData getData();
    void resetMah();

private:
    SHT4xSensor& _sht4;
    int _therm1Pin;
    int _vccPin;
    double _therm1Offset;

    SystemData _currentData;
    SemaphoreHandle_t _dataMutex;

    AdcSnapshot _lastSnapshots[ADC_CH_COUNT];
    uint32_t _lastVoltageUpdateMs;
    uint32_t _lastMahUpdateMs;

    // Physical conversion constants
    const double THERMISTORNOMINAL = 10000.0;
    const double TEMPERATURENOMINAL = 25.0;
    const double BCOEFFICIENT = 3950.0;

    void processAdcSnapshots();
    double calculateBatteryTemp(double ambientTemp, float therm1Mv, float vccMv);
};

#endif // SYSTEM_DATA_MANAGER_H_IMPL
#endif // SYSTEM_DATA_MANAGER_H
