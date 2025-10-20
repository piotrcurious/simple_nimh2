#ifndef INTERNALRESISTANCE_H
#define INTERNALRESISTANCE_H

#include "DataStore.h"
#include "Power.h"

// The InternalResistance class manages the process of measuring the
// battery's internal resistance. It contains a state machine to step
// through the measurement sequence: finding a suitable low current,
// measuring voltage/current pairs, and calculating the final result.
class InternalResistance {
public:
    // --- Constructor ---
    InternalResistance(DataStore* data_store, Power* power);

    // --- Public Methods ---
    void update();              // Main update loop for the IR measurement state machine.
    void start_measurement();   // Initiates the IR measurement process.
    void abort_measurement();   // Aborts the measurement process.

private:
    // --- Private Members ---
    DataStore* _data_store;     // Pointer to the central data repository.
    Power* _power;              // Pointer to the power management module.

    // --- State Machine ---
    unsigned long _state_start_time;
    int _find_min_current_duty_cycle;
    std::vector<ResistanceDataPoint> _measured_pairs;
    void _run_state_machine();

    // --- Private Helper Methods ---
    void _reset();
    bool _perform_linear_regression(float& slope, float& intercept);
};

#endif // INTERNALRESISTANCE_H