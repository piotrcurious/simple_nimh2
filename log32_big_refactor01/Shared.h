#ifndef SHARED_H
#define SHARED_H

#include <cstdint>
#include <vector>
#include <string>
#include <ArduinoEigenDense.h>

// --- Application-wide Enums ---

// Represents the main state of the application
enum class AppState {
    IDLE,
    BUILDING_MODEL,
    MEASURING_IR,
    CHARGING
};

// Represents the currently active screen or display mode
enum class DisplayState {
    MAIN,
    CHARGE_GRAPH,
    IR_GRAPH
};

// Represents the state of the internal resistance measurement process
enum class IRState {
    IDLE,
    START,
    FIND_MIN_CURRENT,
    MEASURE_PAIRS,
    CALCULATE,
    DISPLAY_RESULTS, // Renamed from DISPLAY to avoid macro conflict
    COMPLETE,
    ABORTED
};

// Maps IR remote control hex codes to meaningful names
namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER       = 0xE6,
    KEY_PLAY        = 0x47,
    KEY_INFO        = 0x1F,
    KEY_SOURCE      = 0x01,
    // Add other keys as needed
  };
}


// --- Data Structures ---

// A container for a complete set of sensor readings at a single point in time
struct Measurement {
    float voltage_V = 0.0f;
    float current_A = 0.0f;
    double temp_surface_C = 0.0f;
    double temp_ambient_C = 0.0f;
    double temp_diff_C = 0.0f;
    int duty_cycle = 0;
    unsigned long timestamp_ms = 0;
};

// Holds the coefficients for the polynomial current estimation model
struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool is_built = false;
};

// Stores a single entry in the charge history log
struct ChargeLogEntry {
    unsigned long timestamp_ms;
    float current_A;
    float voltage_V;
    float temp_ambient_C;
    float temp_surface_C;
    int duty_cycle;
    float internal_resistance_ohms;
};

// Holds a pair of voltage/current measurements for calculating internal resistance
struct ResistanceDataPoint {
    float current_A;
    float voltage_V;
};

#endif // SHARED_H