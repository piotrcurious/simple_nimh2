#ifndef DATASTORE_H
#define DATASTORE_H

#include "Shared.h"
#include "config.h"
#include <vector>
#include <TFT_eSPI.h> // For color definitions

// The DataStore class acts as a central repository for all application state,
// sensor readings, and historical data. This helps to eliminate global variables
// and provides a single source of truth for the entire application.
class DataStore {
public:
    // --- Constructor ---
    DataStore();

    // --- State Management ---
    AppState app_state;
    DisplayState display_state;
    IRState ir_state;
    unsigned long display_state_change_time;

    // --- Real-time Sensor Data ---
    Measurement latest_measurement;
    volatile float mAh_charged;

    // --- Current Estimation Model ---
    CurrentModel current_model;

    // --- Data History for Plotting ---
    std::vector<float> temp_surface_history;
    std::vector<float> temp_ambient_history;
    std::vector<float> temp_diff_history;
    std::vector<float> voltage_history;
    std::vector<float> current_history;
    void add_history_point(const Measurement& measurement);

    // --- Charge Logging ---
    std::vector<ChargeLogEntry> charge_log;
    void log_charge_data(const Measurement& measurement, float internal_resistance);

    // --- Internal Resistance (IR) Data ---
    std::vector<ResistanceDataPoint> ir_data_points;
    float ir_slope;
    float ir_intercept;
    void clear_ir_data();

    // --- Public Methods ---
    void reset_mAh();

private:
    // --- Private Helper Methods ---
    void resize_history_vectors();
};

#endif // DATASTORE_H