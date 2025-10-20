#ifndef POWER_H
#define POWER_H

#include "DataStore.h"

// The Power class manages all aspects of power delivery. This includes
// controlling the PWM output, building the current estimation model,
// and running the main battery charging state machine.
class Power {
public:
    // --- Constructor ---
    Power(DataStore* data_store);

    // --- Public Methods ---
    void begin();                               // Initializes PWM hardware.
    void update();                              // Main update loop for the power state machine.
    void set_duty_cycle(int duty_cycle);        // Sets the PWM duty cycle.
    void start_charging();                      // Initiates the charging process.
    void stop_charging();                       // Stops the charging process.
    void start_model_build();                   // Begins the current model estimation process.

private:
    // --- Private Members ---
    DataStore* _data_store;                     // Pointer to the central data repository.

    // --- State Machine for Building the Current Model ---
    int _build_model_step;
    int _build_model_duty_cycle;
    unsigned long _build_model_last_step_time;
    std::vector<float> _model_duty_cycles;
    std::vector<float> _model_currents;
    void _build_current_model_step();

    // --- State Machine for Battery Charging ---
    unsigned long _last_charging_housekeeping_time;
    bool _charge_battery();

    // --- Private Helper Methods ---
    float _estimate_current(int duty_cycle);
};

#endif // POWER_H