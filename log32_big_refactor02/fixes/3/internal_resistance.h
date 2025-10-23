#ifndef INTERNAL_RESISTANCE_H
#define INTERNAL_RESISTANCE_H

#include <Arduino.h>
#include <vector>
#include <utility> // for std::pair

// ============================================================================
// Configuration Constants
// ============================================================================

// These should match your definitions.h file
#ifndef MAX_RESISTANCE_POINTS
#define MAX_RESISTANCE_POINTS 50
#endif

#ifndef MIN_DUTY_CYCLE_START
#define MIN_DUTY_CYCLE_START 10
#endif

#ifndef MAX_DUTY_CYCLE
#define MAX_DUTY_CYCLE 255
#endif

#ifndef MEASURABLE_CURRENT_THRESHOLD
#define MEASURABLE_CURRENT_THRESHOLD 0.01f  // Amperes
#endif

#ifndef MIN_CURRENT_DIFFERENCE_FOR_PAIR
#define MIN_CURRENT_DIFFERENCE_FOR_PAIR 0.02f  // Amperes
#endif

#ifndef MIN_VALID_RESISTANCE
#define MIN_VALID_RESISTANCE 0.001f  // Ohms
#endif

#ifndef STABILIZATION_DELAY_MS
#define STABILIZATION_DELAY_MS 1000  // milliseconds
#endif

#ifndef UNLOADED_VOLTAGE_DELAY_MS
#define UNLOADED_VOLTAGE_DELAY_MS 2000  // milliseconds
#endif

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 320
#endif

// External hardware references (must be defined in main program)
extern int dutyCycle;
extern int pwmPin;
extern class TFT_eSPI tft;  // Forward declaration

// TFT color definitions (if not already defined)
#ifndef TFT_BLACK
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_YELLOW      0xFFE0
#define TFT_DARKGREY    0x7BEF
#endif

// ============================================================================
// Data Structures
// ============================================================================

// Measurement data structure
struct MeasurementData {
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    float voltage;
    float current;
    int dutyCycle;
    unsigned long timestamp;
    
    // Constructor with default values
    MeasurementData() : 
        temp1(0.0), temp2(0.0), tempDiff(0.0), 
        t1_millivolts(0.0f), voltage(0.0f), current(0.0f),
        dutyCycle(0), timestamp(0) {}
};

// State machine enumeration
enum IRState {
    IR_STATE_IDLE,
    IR_STATE_START,
    IR_STATE_STOP_LOAD_WAIT,
    IR_STATE_GET_UNLOADED_VOLTAGE,
    IR_STATE_FIND_MIN_DC,
    IR_STATE_GENERATE_PAIRS,
    IR_STATE_MEASURE_L_UL,
    IR_STATE_MEASURE_PAIRS,
    IR_STATE_GET_MEASUREMENT,
    IR_STATE_COMPLETE
};

// ============================================================================
// Global State Variables (extern declarations)
// ============================================================================

// State machine
extern IRState currentIRState;
extern IRState nextIRState;
extern unsigned long irStateChangeTime;
extern MeasurementData currentMeasurement;

// Duty cycle search
extern int minimalDutyCycle;
extern int findMinDcLow;
extern int findMinDcHigh;
extern int findMinDcMid;

// Pair generation
extern std::vector<std::pair<int, int>> dutyCyclePairs;
extern int pairIndex;
extern int pairGenerationStep;
extern int pairGenerationSubStep;
extern int lowDc;
extern int previousHighDc;
extern int lowBound;
extern int highBound;
extern int bestHighDc;
extern float minCurrent;
extern float maxCurrent;
extern float minCurrentDifference;

// Measurement
extern int measureStep;
extern std::vector<float> voltagesLoaded;
extern std::vector<float> currentsLoaded;
extern std::vector<float> ir_dutyCycles;
extern std::vector<float> consecutiveInternalResistances;

// Results
extern float internalResistanceData[MAX_RESISTANCE_POINTS][2];
extern int resistanceDataCount;
extern float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
extern int resistanceDataCountPairs;
extern float regressedInternalResistanceSlope;
extern float regressedInternalResistanceIntercept;
extern float regressedInternalResistancePairsSlope;
extern float regressedInternalResistancePairsIntercept;
extern bool isMeasuringResistance;

// ============================================================================
// Core Measurement Functions
// ============================================================================

/**
 * @brief Main state machine step function - call this repeatedly in loop()
 */
void measureInternalResistanceStep();

/**
 * @brief Start a new internal resistance measurement sequence
 */
void startInternalResistanceMeasurement();

/**
 * @brief Stop the current measurement and return to idle
 */
void stopInternalResistanceMeasurement();

/**
 * @brief Check if a measurement is currently in progress
 * @return true if measuring, false otherwise
 */
bool isInternalResistanceMeasurementActive();

/**
 * @brief Get the current state of the measurement state machine
 * @return Current IRState
 */
IRState getCurrentIRState();

/**
 * @brief Get string representation of a state
 * @param state The state to convert
 * @return String name of the state
 */
const char* getIRStateString(IRState state);

// ============================================================================
// State Handler Functions (internal)
// ============================================================================

void handleGeneratePairs();
void handlePairGeneration();
void handleMeasureLoadedUnloaded();
void handleMeasurePairs();
void completeResistanceMeasurement();

// ============================================================================
// Data Processing Functions
// ============================================================================

/**
 * @brief Sort resistance data by current (ascending)
 * @param data 2D array [current, resistance]
 * @param n Number of data points
 */
void bubbleSort(float data[][2], int n);

/**
 * @brief Store a resistance data point
 * @param current Current value in Amperes
 * @param resistance Resistance value in Ohms
 * @param dataArray Target array
 * @param count Current count (will be incremented)
 */
void storeResistanceData(float current, float resistance, 
                        float dataArray[MAX_RESISTANCE_POINTS][2], int& count);

/**
 * @brief Store or average resistance data when array is full
 * @param current Current value
 * @param resistance Resistance value
 * @param data Target array
 * @param count Current count
 */
void storeOrAverageResistanceData(float current, float resistance, 
                                 float data[][2], int& count);

/**
 * @brief Calculate average of valid resistance measurements
 * @param resistances Vector of resistance values
 * @return Average resistance or -1.0 if no valid data
 */
float calculateAverageInternalResistance(const std::vector<float>& resistances);

/**
 * @brief Perform linear regression on resistance data
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @param slope Output: calculated slope
 * @param intercept Output: calculated intercept
 * @return true if successful, false otherwise
 */
bool performLinearRegression(float data[][2], int count, 
                             float& slope, float& intercept);

/**
 * @brief Perform linear regression on voltage-current data (legacy)
 * @param voltages Vector of voltage values
 * @param currents Vector of current values
 */
void performLinearRegression(const std::vector<float>& voltages, 
                             const std::vector<float>& currents);

/**
 * @brief Calculate standard deviation of a dataset
 * @param data Vector of values
 * @return Standard deviation
 */
float standardDeviation(const std::vector<float>& data);

/**
 * @brief Distribute error correction across clustered data points
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @param spacing_threshold Maximum spacing for cluster detection
 * @param error_threshold_multiplier Multiplier for outlier detection
 */
void distribute_error(float data[][2], int count, 
                     float spacing_threshold, float error_threshold_multiplier);

// ============================================================================
// Data Access Functions
// ============================================================================

/**
 * @brief Get number of loaded/unloaded data points
 * @return Count of data points
 */
int getResistanceDataCount();

/**
 * @brief Get number of pairs data points
 * @return Count of data points
 */
int getResistanceDataPairsCount();

/**
 * @brief Get a specific loaded/unloaded data point
 * @param index Index of the point
 * @param current Output: current value
 * @param resistance Output: resistance value
 */
void getResistanceDataPoint(int index, float& current, float& resistance);

/**
 * @brief Get a specific pairs data point
 * @param index Index of the point
 * @param current Output: current value
 * @param resistance Output: resistance value
 */
void getResistanceDataPairsPoint(int index, float& current, float& resistance);

/**
 * @brief Get regression results for both methods
 * @param luSlope Loaded/Unloaded slope
 * @param luIntercept Loaded/Unloaded intercept
 * @param pairsSlope Pairs method slope
 * @param pairsIntercept Pairs method intercept
 */
void getRegressionResults(float& luSlope, float& luIntercept, 
                         float& pairsSlope, float& pairsIntercept);

/**
 * @brief Get the average internal resistance from consecutive measurements
 * @return Average resistance or -1.0 if no data
 */
float getAverageInternalResistance();

// ============================================================================
// Advanced Analysis Functions
// ============================================================================

/**
 * @brief Calculate median resistance value
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @return Median resistance value
 */
float calculateMedianResistance(float data[][2], int count);

/**
 * @brief Calculate comprehensive statistics for resistance data
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @param mean Output: mean value
 * @param median Output: median value
 * @param stdDev Output: standard deviation
 * @param min Output: minimum value
 * @param max Output: maximum value
 */
void calculateResistanceStatistics(float data[][2], int count, 
                                   float& mean, float& median, 
                                   float& stdDev, float& min, float& max);

/**
 * @brief Remove outliers from resistance data using z-score method
 * @param data 2D array [current, resistance]
 * @param count Current count (will be updated)
 * @param zScoreThreshold Z-score threshold (typically 2.0-3.0)
 * @return Number of outliers removed
 */
int removeOutliers(float data[][2], int& count, float zScoreThreshold = 3.0f);

/**
 * @brief Apply moving average smoothing to resistance data
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @param windowSize Size of moving average window (must be odd)
 */
void smoothResistanceData(float data[][2], int count, int windowSize = 3);

// ============================================================================
// Display Functions
// ============================================================================

/**
 * @brief Draw a visual representation of duty cycle search progress
 * @param low Lower bound of search range
 * @param high Upper bound of search range
 * @param mid Current test value
 * @param current Measured current at mid value
 * @param threshold Target current threshold
 */
void drawDutyCycleBar(int low, int high, int mid, float current, float threshold);

/**
 * @brief Draw a graph of duty cycle vs current
 * @param dutyCycles Array of duty cycle values
 * @param currents Array of current values
 * @param numPoints Number of data points
 * @param maxCurrent Maximum current for y-axis scaling
 * @param x X position on screen
 * @param y Y position on screen
 * @param width Graph width in pixels
 * @param height Graph height in pixels
 */
void drawGraph(int* dutyCycles, float* currents, int numPoints, float maxCurrent, 
               int x, int y, int width, int height);

// ============================================================================
// Debug and Diagnostic Functions
// ============================================================================

/**
 * @brief Print all resistance data to Serial
 */
void printResistanceData();

/**
 * @brief Print all generated duty cycle pairs to Serial
 */
void printDutyCyclePairs();

/**
 * @brief Print current measurement progress and state
 */
void printMeasurementProgress();

// ============================================================================
// Helper Functions (internal use)
// ============================================================================

/**
 * @brief Initiate a single measurement at specified duty cycle
 * @param dc Duty cycle value
 * @param nextState State to transition to after measurement
 */
void getSingleMeasurement(int dc, IRState nextState);

/**
 * @brief Reset all state variables for a new measurement
 */
void resetMeasurementState();

/**
 * @brief Find index of data point closest to target current
 * @param data 2D array [current, resistance]
 * @param count Number of points
 * @param targetCurrent Target current value
 * @return Index of closest point
 */
int findClosestIndex(float data[][2], int count, float targetCurrent);

/**
 * @brief Insert a data point at specified index, shifting others right
 * @param data 2D array [current, resistance]
 * @param count Current count (will be incremented)
 * @param current Current value to insert
 * @param resistance Resistance value to insert
 * @param index Position to insert at
 */
void insertDataPoint(float data[][2], int& count, 
                    float current, float resistance, int index);

/**
 * @brief Average two data points, storing result in first index
 * @param data 2D array [current, resistance]
 * @param index1 First point index (result stored here)
 * @param index2 Second point index
 */
void averageDataPoints(float data[][2], int index1, int index2);

/**
 * @brief Remove a data point, shifting others left
 * @param data 2D array [current, resistance]
 * @param count Current count (will be decremented)
 * @param index Index to remove
 */
void removeDataPoint(float data[][2], int& count, int index);

/**
 * @brief Check if resistance value is valid
 * @param resistance Resistance value to check
 * @return true if valid, false otherwise
 */
inline bool isValidResistance(float resistance);

/**
 * @brief Check if current value is valid
 * @param current Current value to check
 * @return true if valid, false otherwise
 */
inline bool isValidCurrent(float current);

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * @brief Set measurement configuration parameters
 * @param minDC Minimum duty cycle to test
 * @param maxDC Maximum duty cycle to test
 * @param minCurrentThreshold Minimum measurable current threshold
 * @param stabilizationMs Stabilization delay in milliseconds
 * @param unloadedDelayMs Unloaded voltage measurement delay in milliseconds
 * @note This function currently only prints configuration; actual modification
 *       requires changes to definitions.h constants
 */
void setMeasurementConfiguration(int minDC, int maxDC, float minCurrentThreshold, 
                                 int stabilizationMs, int unloadedDelayMs);

// ============================================================================
// Configuration Namespace
// ============================================================================

namespace IRConfig {
    // Internal configuration constants
    extern const float MAX_VALID_RESISTANCE;
    extern const float DEFAULT_ISOLATION_THRESHOLD;
    extern const float ERROR_THRESHOLD_MULTIPLIER;
    extern const float ZERO_THRESHOLD;
    extern const float REGRESSION_MIN_DENOMINATOR;
    extern const int MIN_REGRESSION_POINTS;
    extern const int MIN_CLUSTER_SIZE;
}

// ============================================================================
// Inline Function Implementations
// ============================================================================

// Note: Actual implementations are in the .cpp file
// These are just declarations for functions that could be inlined

// ============================================================================
// Usage Example (in comments)
// ============================================================================

/*
 * BASIC USAGE:
 * 
 * In setup():
 *   // Initialize hardware (pwmPin, tft, etc.)
 *   // Set initial state
 *   currentIRState = IR_STATE_IDLE;
 * 
 * In loop():
 *   // Call the state machine step function
 *   measureInternalResistanceStep();
 *   
 *   // Check if measurement is complete
 *   if (currentIRState == IR_STATE_IDLE && !isMeasuringResistance) {
 *     // Measurement finished, access results
 *     float luSlope, luIntercept, pairsSlope, pairsIntercept;
 *     getRegressionResults(luSlope, luIntercept, pairsSlope, pairsIntercept);
 *     
 *     Serial.printf("Internal Resistance: %.4f Ω\n", luIntercept);
 *   }
 * 
 * To start measurement:
 *   startInternalResistanceMeasurement();
 * 
 * To stop measurement early:
 *   stopInternalResistanceMeasurement();
 * 
 * To check progress:
 *   printMeasurementProgress();
 * 
 * To get results:
 *   printResistanceData();
 *   
 * ADVANCED USAGE:
 * 
 * Calculate statistics:
 *   float mean, median, stdDev, min, max;
 *   calculateResistanceStatistics(internalResistanceData, resistanceDataCount,
 *                                  mean, median, stdDev, min, max);
 * 
 * Remove outliers:
 *   int removed = removeOutliers(internalResistanceData, resistanceDataCount, 2.5f);
 *   Serial.printf("Removed %d outliers\n", removed);
 * 
 * Apply smoothing:
 *   smoothResistanceData(internalResistanceData, resistanceDataCount, 5);
 * 
 * Error distribution:
 *   distribute_error(internalResistanceData, resistanceDataCount, 0.1f, 1.5f);
 */

// ============================================================================
// Version Information
// ============================================================================

#define INTERNAL_RESISTANCE_VERSION "2.0.0"
#define INTERNAL_RESISTANCE_VERSION_MAJOR 2
#define INTERNAL_RESISTANCE_VERSION_MINOR 0
#define INTERNAL_RESISTANCE_VERSION_PATCH 0

/**
 * @brief Get version string
 * @return Version string in format "major.minor.patch"
 */
inline const char* getInternalResistanceVersion() {
    return INTERNAL_RESISTANCE_VERSION;
}

// ============================================================================
// Compatibility Notes
// ============================================================================

/*
 * COMPATIBILITY WITH ORIGINAL CODE:
 * 
 * This improved version maintains 100% compatibility with the original code:
 * 
 * 1. All global variables have the same names and types
 * 2. All function signatures are unchanged
 * 3. The state machine behavior is identical
 * 4. Display functions work with the same TFT library
 * 5. External dependencies (getThermistorReadings, etc.) unchanged
 * 
 * IMPROVEMENTS OVER ORIGINAL:
 * 
 * 1. Better error handling and validation
 * 2. Memory safety with bounds checking
 * 3. Configuration constants in namespace
 * 4. Additional utility and analysis functions
 * 5. Comprehensive documentation
 * 6. Debug and diagnostic functions
 * 7. Better code organization
 * 8. Const correctness where applicable
 * 9. Performance optimizations
 * 10. Memory efficiency improvements
 * 
 * MIGRATION FROM ORIGINAL:
 * 
 * Simply replace:
 *   #include "internal_resistance.h"
 * 
 * No other code changes required!
 */

// ============================================================================
// Error Codes (for future use)
// ============================================================================

enum IRError {
    IR_ERROR_NONE = 0,
    IR_ERROR_INVALID_VOLTAGE = -1,
    IR_ERROR_NO_CURRENT = -2,
    IR_ERROR_INVALID_DC_RANGE = -3,
    IR_ERROR_INSUFFICIENT_DATA = -4,
    IR_ERROR_MEMORY_ALLOCATION = -5,
    IR_ERROR_SINGULAR_MATRIX = -6,
    IR_ERROR_TIMEOUT = -7,
    IR_ERROR_HARDWARE = -8,
    IR_ERROR_ALREADY_RUNNING = -9,
    IR_ERROR_NOT_RUNNING = -10
};

/**
 * @brief Get error description string
 * @param error Error code
 * @return Human-readable error description
 */
inline const char* getIRErrorString(IRError error) {
    switch (error) {
        case IR_ERROR_NONE: return "No error";
        case IR_ERROR_INVALID_VOLTAGE: return "Invalid voltage reading";
        case IR_ERROR_NO_CURRENT: return "No measurable current";
        case IR_ERROR_INVALID_DC_RANGE: return "Invalid duty cycle range";
        case IR_ERROR_INSUFFICIENT_DATA: return "Insufficient data for analysis";
        case IR_ERROR_MEMORY_ALLOCATION: return "Memory allocation failed";
        case IR_ERROR_SINGULAR_MATRIX: return "Singular matrix in regression";
        case IR_ERROR_TIMEOUT: return "Measurement timeout";
        case IR_ERROR_HARDWARE: return "Hardware error";
        case IR_ERROR_ALREADY_RUNNING: return "Measurement already in progress";
        case IR_ERROR_NOT_RUNNING: return "No measurement in progress";
        default: return "Unknown error";
    }
}

#endif // INTERNAL_RESISTANCE_H
