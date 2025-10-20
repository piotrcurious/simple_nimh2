#include "DataStore.h"
#include "config.h"

// --- Constructor ---
// Initializes the DataStore with default values.
DataStore::DataStore() :
    app_state(AppState::IDLE),
    display_state(DisplayState::MAIN),
    ir_state(IRState::IDLE),
    display_state_change_time(0),
    mAh_charged(0.0f),
    ir_slope(0.0f),
    ir_intercept(0.0f)
{
    // Pre-allocate memory for the history vectors to avoid reallocations
    // in the main loop.
    temp_surface_history.reserve(MAX_DATA_POINTS);
    temp_ambient_history.reserve(MAX_DATA_POINTS);
    temp_diff_history.reserve(MAX_DATA_POINTS);
    voltage_history.reserve(MAX_DATA_POINTS);
    current_history.reserve(MAX_DATA_POINTS);

    // Initialize history vectors with default values
    resize_history_vectors();
}

// --- Public Methods ---

// Adds a new measurement to the history vectors used for plotting.
// It maintains a fixed-size window of the most recent data points.
void DataStore::add_history_point(const Measurement& measurement) {
    if (voltage_history.size() >= MAX_DATA_POINTS) {
        voltage_history.erase(voltage_history.begin());
        current_history.erase(current_history.begin());
        temp_surface_history.erase(temp_surface_history.begin());
        temp_ambient_history.erase(temp_ambient_history.begin());
        temp_diff_history.erase(temp_diff_history.begin());
    }
    voltage_history.push_back(measurement.voltage_V);
    current_history.push_back(measurement.current_A);
    temp_surface_history.push_back(measurement.temp_surface_C);
    temp_ambient_history.push_back(measurement.temp_ambient_C);
    temp_diff_history.push_back(measurement.temp_diff_C);
}

// Logs a complete set of data for the detailed charging graph.
void DataStore::log_charge_data(const Measurement& measurement, float internal_resistance) {
    charge_log.push_back({
        measurement.timestamp_ms,
        measurement.current_A,
        measurement.voltage_V,
        (float)measurement.temp_ambient_C,
        (float)measurement.temp_surface_C,
        measurement.duty_cycle,
        internal_resistance
    });
}

// Resets the accumulated charge counter.
void DataStore::reset_mAh() {
    mAh_charged = 0.0f;
}

// Clears all data related to internal resistance measurements.
void DataStore::clear_ir_data() {
    ir_data_points.clear();
    ir_slope = 0.0f;
    ir_intercept = 0.0f;
}


// --- Private Helper Methods ---

// Initializes or resets the history vectors to a default state.
void DataStore::resize_history_vectors() {
    temp_surface_history.assign(MAX_DATA_POINTS, 25.0f);
    temp_ambient_history.assign(MAX_DATA_POINTS, 25.0f);
    temp_diff_history.assign(MAX_DATA_POINTS, 0.0f);
    voltage_history.assign(MAX_DATA_POINTS, 1.0f);
    current_history.assign(MAX_DATA_POINTS, 0.0f);
}