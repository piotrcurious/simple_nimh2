#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "LGFX_Config.h"
#include <SPI.h>

#include <cmath>
#include <limits>
#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <ArduinoEigenDense.h>
#include <cstdlib>
#include <map>
#include <ctime>
#include <Arduino.h>

#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
#include "analog.h"

#define DEBUG_LABELS

// --- Pin Definitions ---
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define THERMISTOR_PIN_1 36
#define THERMISTOR_PIN_1_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_PIN_1_OVERSAMPLING 16
#define THERMISTOR_VCC_PIN 35
#define THERMISTOR_VCC_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_VCC_OVERSAMPLING 16
#define VOLTAGE_READ_PIN 39
#define VOLTAGE_ATTENUATION ADC_ATTEN_DB_11
#define VOLTAGE_OVERSAMPLING 16
#define CURRENT_SHUNT_PIN 34
#define CURRENT_SHUNT_ATTENUATION ADC_ATTEN_DB_0
#define CURRENT_SHUNT_OVERSAMPLING 16
#define PWM_PIN 19
#define IR_RECEIVE_PIN 15

// --- Constants ---
#define CHARGING_UPDATE_INTERVAL_MS 2000
#define PWM_MAX 255
#define TOTAL_TIMEOUT (20UL * 60 * 60 * 1000)
#define PLOT_UPDATE_INTERVAL_MS 5000
#define PLOT_DATA_UPDATE_INTERVAL 1000
#define CHARGING_HOUSEKEEP_INTERVAL 500
#define IR_HANDLE_INTERVAL_MS 500
#define MAIN_VCC_RATIO 2.0
#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 75
#define PWM_FREQUENCY 1000
#define BUILD_CURRENT_MODEL_DELAY 250

// Physical defaults
static const float DEFAULT_CELL_MASS_KG       = 0.013f;
static const float DEFAULT_SPECIFIC_HEAT      = 1000.0f;
static const float DEFAULT_SURFACE_AREA_M2    = 0.001477f;
static const float DEFAULT_CONVECTIVE_H       = 0.15f;
static const float DEFAULT_EMISSIVITY         = 0.9f;
static const float STEFAN_BOLTZMANN           = 5.670374419e-8f;

// Charging constants
const float MAX_TEMP_DIFF_THRESHOLD = 0.5f;
const uint8_t OVERTEMP_TRIP_TRESHOLD = 3;
extern float maximumCurrent;
const float MH_ELECTRODE_RATIO = 0.60f;
const uint32_t CHARGE_EVALUATION_INTERVAL_MS = 120000;
const int CHARGE_CURRENT_STEP = 1;
const int MAX_CHARGE_DUTY_CYCLE = 254;
const int MIN_CHARGE_DUTY_CYCLE = 5;
#define ISOLATION_THRESHOLD 0.02f

// R_int measurement constants
const float MEASURABLE_CURRENT_THRESHOLD = 0.005f;
const int MIN_DUTY_CYCLE_START = 8;
const int MAX_DUTY_CYCLE = 255;
const int DUTY_CYCLE_INCREMENT_FIND_MIN = 5;
const int STABILIZATION_DELAY_MS = 2000;
const int STABILIZATION_PAIRS_FIND_DELAY_MS = 1000;
const int UNLOADED_VOLTAGE_DELAY_MS = 6000;
const int MIN_DUTY_CYCLE_ADJUSTMENT_STEP = 5;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.02f;
const float MIN_VALID_RESISTANCE = 0.0f;
const int MAX_RESISTANCE_POINTS = 100;
const float OUTLIER_THRESHOLD_STD_DEV = 2.0f;
const float WARM_START_TARGET_CURRENT = 0.150f;
const int WARM_START_DC_SEARCH_RANGE = 10;
const int EXPLORATORY_DC_OFFSET = 15;

// Plotting parameters
#define PLOT_WIDTH          320
#define PLOT_HEIGHT         (216 - 3)
#define PLOT_X_START        0
#define PLOT_Y_START        0
#define LABEL_Y_START       (PLOT_Y_START + PLOT_HEIGHT + 3)
#define LABEL_TEXT_SIZE     1

// Plot colors
#define PLOT_X_AXIS_COLOR   TFT_WHITE
#define PLOT_Y_AXIS_COLOR   TFT_WHITE
#define PLOT_ZERO_COLOR     0x62ec
#define GRAPH_COLOR_1       TFT_RED
#define GRAPH_COLOR_2       TFT_GREEN
#define GRAPH_COLOR_DIFF    TFT_BLUE
#define GRAPH_COLOR_VOLTAGE TFT_YELLOW
#define GRAPH_COLOR_CURRENT TFT_MAGENTA
#define GRAPH_COLOR_RESISTANCE TFT_ORANGE
#define GRAPH_COLOR_RESISTANCE_PAIR TFT_CYAN

// Fixed plot ranges
const float MIN_TEMP             = 15.0;
const float MAX_TEMP             = 30.0;
const float MIN_DIFF_TEMP        = -0.5;
extern float MAX_DIFF_TEMP;
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 2.3f;
const float MIN_CURRENT = 0.0f;
const float MAX_CURRENT = 0.40f;

// --- Enums and Structs ---

enum DisplayState {
    DISPLAY_STATE_IDLE,
    DISPLAY_STATE_MAIN,
    DISPLAY_STATE_IR_GRAPH,
    DISPLAY_STATE_CHARGE_GRAPH
};

enum AppState {
    APP_STATE_IDLE,
    APP_STATE_BUILDING_MODEL,
    APP_STATE_MEASURING_IR,
    APP_STATE_CHARGING
};

struct MeasurementData {
    float voltage;
    float current;
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    int dutyCycle;
    unsigned long timestamp;
};

enum class LabelVerticalPlacement {
    CENTER,
    ABOVE,
    BELOW
};

struct Label {
    std::string text;
    int x;
    int y;
    uint16_t color;
    int textWidth;
    int textHeight;
    int lineStartX;
    int lineEndX;
    int y_initial;
    int lineY;
    LabelVerticalPlacement verticalPlacement;
    float minValue;
    float maxValue;
};

struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool isModelBuilt = false;
};

struct TickLabel {
    int y;
    float value;
    uint16_t color;
    Label label;
};

struct ChargeLogData {
    unsigned long timestamp;
    float current;
    float voltage;
    float ambientTemperature;
    float batteryTemperature;
    int dutyCycle;
    float internalResistanceLoadedUnloaded;
    float internalResistancePairs;
};

struct MHElectrodeData {
    float unloadedVoltage;
    float loadedVoltage;
    float targetVoltage;
    float voltageDifference;
    float current;
    uint32_t dutyCycle;
    uint32_t timestamp;
};

enum ChargingState {
    CHARGE_IDLE = 0,
    CHARGE_FIND_OPT,
    CHARGE_WAIT_IR,
    CHARGE_MONITOR,
    CHARGE_STOPPED
};

enum FindPhase {
    FIND_IDLE,
    FIND_INIT_HIGHDC,
    FIND_BINARY_PREPARE,
    FIND_BINARY_WAIT,
    FIND_EXPLORE,
    FIND_EXPLORE_WAIT,
    FIND_FINAL_WAIT,
    FIND_COMPLETE
};

struct FindOptManager {
    bool active = false;
    int maxDC = 0;
    int lowDC = 0;
    int highDC = 0;
    int optimalDC = MIN_CHARGE_DUTY_CYCLE;
    float closestVoltageDifference = 1000.0f;
    float targetVoltage = 0.0f;
    float initialUnloadedVoltage = 0.0f;
    std::vector<MHElectrodeData> cache;
    FindPhase phase = FIND_IDLE;
    bool isReevaluation = false;
    int exploration_step = 0;
};

struct RemeasureManager {
    int remeasure_index = 0;
    float remeasure_low_voltage = 0.0f;
    float remeasure_low_current = 0.0f;
    int remeasure_sub_step = 0;
    float target_current = 0.0f;
    int remeasure_low_bound_dc = 0;
    int remeasure_high_bound_dc = 0;
    int remeasure_mid_dc = 0;
    int best_remeasure_dc = 0;
    float min_current_diff_remeasure = 1e9f;
};

enum MeasState {
    MEAS_IDLE,
    MEAS_STOPLOAD_WAIT,
    MEAS_SAMPLE_UNLOADED,
    MEAS_APPLY_LOAD,
    MEAS_SAMPLE_LOADED,
    MEAS_COMPLETE,
    MEAS_ABORTED
};

struct AsyncMeasure {
    MeasState state = MEAS_IDLE;
    int testDuty = 0;
    unsigned long stateStart = 0;
    unsigned long unloadedDelay = 0;
    unsigned long stabilizationDelay = 0;
    MeasurementData unloadedData;
    MeasurementData loadedData;
    MHElectrodeData result;
    bool resultReady = false;
    bool active() const { return state != MEAS_IDLE && state != MEAS_COMPLETE && state != MEAS_ABORTED; }
    void reset() {
        state = MEAS_IDLE;
        testDuty = 0;
        stateStart = 0;
        unloadedDelay = 0;
        stabilizationDelay = 0;
        resultReady = false;
    }
};

namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0xE6,
    KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
    KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
    KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
    KEY_OK = 0x68, KEY_MENU = 0x79, KEY_RED = 0x6c, KEY_GREEN = 0x14,
    KEY_YELLOW = 0x15, KEY_BLUE = 0x16, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
    KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10, KEY_REWIND = 0x45, KEY_PLAY = 0x47,
    KEY_PAUSE = 0x4A, KEY_FORWARD = 0x48, KEY_STOP = 0x46, KEY_SETTINGS = 0x1A,
    KEY_INFO = 0x1F, KEY_SUBTITLES = 0x25, KEY_MUTE = 0x0F, KEY_NETFLIX = 0xF3,
    KEY_PRIME_VIDEO = 0xF4, KEY_GUIDE = 0x4F, KEY_SOURCE = 0x01
  };
}

// --- Extern Global Variables ---
extern LGFX tft;
extern SHT4xSensor sht4Sensor;
extern ThermistorSensor thermistorSensor;
extern CurrentModel currentModel;
extern AsyncMeasure meas;
extern FindOptManager findOpt;

extern float temp1_values[PLOT_WIDTH];
extern float temp2_values[PLOT_WIDTH];
extern float diff_values[PLOT_WIDTH];
extern float voltage_values[PLOT_WIDTH];
extern float current_values[PLOT_WIDTH];

extern volatile float voltage_mv;
extern volatile float current_ma;
extern volatile double mAh_charged;
extern volatile bool resetAh;
extern volatile uint32_t mAh_last_time;

extern uint32_t dutyCycle;
extern bool isCharging;
extern bool isMeasuringResistance;
extern ChargingState chargingState;
extern int cachedOptimalDuty;

extern float internalResistanceData[MAX_RESISTANCE_POINTS][2];
extern int resistanceDataCount;
extern float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
extern int resistanceDataCountPairs;
extern float regressedInternalResistanceSlope;
extern float regressedInternalResistanceIntercept;
extern float regressedInternalResistancePairsSlope;
extern float regressedInternalResistancePairsIntercept;
extern std::vector<ChargeLogData> chargeLog;

extern unsigned long lastPlotUpdateTime;
extern unsigned long lastChargingHouseTime;
extern unsigned long lastIRHandleTime;
extern DisplayState currentDisplayState;
extern uint8_t overtemp_trip_counter;
extern unsigned long chargePhaseStartTime;
extern unsigned long chargingStartTime;
extern unsigned long lastChargeEvaluationTime;
extern const int pwmPin;
extern const int pwmResolutionBits;
extern const int pwmMaxDutyCycle;
extern double THERMISTOR_1_OFFSET;


// --- Function Declarations ---
// These are functions that are defined in one .cpp file but called from another.
// This helps avoid circular dependencies.

// from main .ino
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
void buildCurrentModel(bool warmStart);
float estimateCurrent(int dutyCycle);
MeasurementData takeMeasurement(int dc, uint32_t stabilization_delay);
void processThermistorData(const MeasurementData& data, const String& measurementType);
inline unsigned long unmanagedCastUL(unsigned long v){ return v; }


// from graphing.cpp
void drawChargePlot(bool autoscaleX, bool autoscaleY);
void plotTemperatureData();
void prepareTemperaturePlot();
void plotVoltageData();
void displayInternalResistanceGraph();
void displayTemperatureLabels(double temp1, double temp2, double tempDiff, float t1_millivolts, float voltage, float current);
void updateTemperatureHistory(double temp1, double temp2, double tempDiff, float voltage, float current);

// from internal_resistance.cpp
void measureInternalResistance();
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept);
void bubbleSort(float data[][2], int n);
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count);
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier);


// from charging.cpp
void startCharging();
void stopCharging();
void handleBatteryCharging();
bool chargeBattery();
void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay, unsigned long unloaded_delay);
bool measurementStep();
bool fetchMeasurementResult(MHElectrodeData &out);
void abortMeasurement();
void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval);
bool findOptimalChargingDutyCycleStepAsync();
float estimateTempDiff(float voltageUnderLoad, float voltageNoLoad, float current, float internalResistanceParam, float ambientTempC, uint32_t currentTime, uint32_t lastChargeEvaluationTime, float BatteryTempC, float cellMassKg, float specificHeat, float area, float convectiveH, float emissivity);


#endif // DEFINITIONS_H
