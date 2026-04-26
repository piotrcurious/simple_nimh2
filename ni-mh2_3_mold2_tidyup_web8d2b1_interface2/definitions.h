#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#ifndef MOCK_TEST
#include <SPI.h>
#endif

#include <cmath>
#include <limits>
#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#ifndef MOCK_TEST
#include <ArduinoEigenDense.h>
#else
#include "test_mock/ArduinoEigenDense.h"
#endif
#include <cstdlib>
#include <map>
#include <ctime>
#ifndef MOCK_TEST
#include <Arduino.h>
#else
#include "test_mock/Arduino.h"
#endif

#include "SHT4xSensor.h"
#include "SystemDataManager.h"
#ifndef MOCK_TEST
#include "analog.h"
#endif

#define DEBUG_LABELS

// --- Pin Definitions ---
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
#define CURRENT_SHUNT_ATTENUATION ADC_ATTEN_DB_11
#define CURRENT_SHUNT_OVERSAMPLING 16
#define PWM_PIN 19

// --- Constants ---
#define CHARGING_UPDATE_INTERVAL_MS 2000
#define PWM_MAX 255
#define TOTAL_TIMEOUT (20UL * 60 * 60 * 1000)
#define PLOT_UPDATE_INTERVAL_MS 5000
#define PLOT_DATA_UPDATE_INTERVAL 1000
#define CHARGING_HOUSEKEEP_INTERVAL 500
#define MAIN_VCC_RATIO 2.0
#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 75
#define PWM_FREQUENCY 1000
#define BUILD_CURRENT_MODEL_DELAY 250

// Physical defaults
static const float DEFAULT_CELL_MASS_KG       = 0.014f;
static const float DEFAULT_SPECIFIC_HEAT      = 1000.0f;
static const float DEFAULT_SURFACE_AREA_M2    = 0.001477f;
static const float DEFAULT_CONVECTIVE_H       = 0.09f;
static const float DEFAULT_EMISSIVITY         = 0.9f;
static const float STEFAN_BOLTZMANN           = 5.670374419e-8f;

// Charging constants
const float MAX_TEMP_DIFF_THRESHOLD = 0.5f;
const uint8_t OVERTEMP_TRIP_TRESHOLD = 3;
extern float maximumCurrent;
const float MH_ELECTRODE_RATIO = 0.7f;
const uint32_t CHARGE_EVALUATION_INTERVAL_MS = 240000;
const int CHARGE_CURRENT_STEP = 1;
const int MAX_CHARGE_DUTY_CYCLE = 250;
const int MIN_CHARGE_DUTY_CYCLE = 10;
#define ISOLATION_THRESHOLD 0.02f

// R_int measurement constants
extern float MEASURABLE_CURRENT_THRESHOLD;
const int MIN_DUTY_CYCLE_START = 8;
const int MAX_DUTY_CYCLE = 255;
const int DUTY_CYCLE_INCREMENT_FIND_MIN = 4;
const int STABILIZATION_DELAY_MS = 1000;
const int STABILIZATION_PAIRS_FIND_DELAY_MS = 1000;
const int UNLOADED_VOLTAGE_DELAY_MS = 2000;
const int MIN_DUTY_CYCLE_ADJUSTMENT_STEP = 1;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.08f;
const float MIN_VALID_RESISTANCE = 0.0f;
const int MAX_RESISTANCE_POINTS = 100;

// Plotting parameters (now used for memory buffers only)
#define PLOT_WIDTH          120

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

enum class BuildModelPhase { Idle = 0, Settle, Calibrate, DetectDeadRegion, SetDuty, WaitMeasurement, Finish };

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

struct MeasurementData {
    float voltage;
    float current;
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    uint32_t timestamp;
    uint8_t dutyCycle;
};

struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool isModelBuilt = false;
};

struct ChargeLogData {
    uint32_t timestamp;
    float current;
    float voltage;
    float ambientTemperature;
    float batteryTemperature;
    uint8_t dutyCycle;
    float internalResistanceLoadedUnloaded;
    float internalResistancePairs;
};

struct MHElectrodeData {
    float unloadedVoltage;
    float loadedVoltage;
    float targetVoltage;
    float voltageDifference;
    float current;
    uint32_t timestamp;
    uint8_t dutyCycle;
};

enum ChargingState {
    CHARGE_IDLE = 0,
    CHARGE_FIND_OPT,
    CHARGE_MONITOR,
    CHARGE_STOPPED
};

enum FindPhase {
    FIND_IDLE,
    FIND_INIT_HIGHDC,
    FIND_BINARY_PREPARE,
    FIND_BINARY_WAIT,
    RE_EVAL_START,
    RE_EVAL_DETECT_OUTLIERS,
    RE_EVAL_CORRECTIVE_MEASUREMENT_PREPARE,
    RE_EVAL_CORRECTIVE_MEASUREMENT_WAIT,
    RE_EVAL_EXPLORATORY_MEASUREMENT_PREPARE,
    RE_EVAL_EXPLORATORY_MEASUREMENT_WAIT,
    RE_EVAL_FINISH,
    FIND_FINAL_WAIT,
    FIND_COMPLETE
};

struct OutlierInfo {
    int original_index;
    float current;
    float resistance;
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
    std::vector<OutlierInfo> outliers;
    int outlier_measurement_index = 0;
    int exploratory_measurement_phase = 0; // 0 for low, 1 for high
};

enum RemeasurePhase {
    REMEASURE_IDLE,
    REMEASURE_BINARY_SEARCH_PREPARE,
    REMEASURE_BINARY_SEARCH_WAIT,
    REMEASURE_COMPLETE
};

struct RemeasureManager {
    bool active = false;
    float targetCurrent = 0.0f;
    int lowDC = MIN_CHARGE_DUTY_CYCLE;
    int highDC = MAX_CHARGE_DUTY_CYCLE;
    RemeasurePhase phase = REMEASURE_IDLE;
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

// --- Extern Global Variables ---
extern SHT4xSensor sht4Sensor;
extern SystemDataManager systemData;
extern CurrentModel currentModel;
extern AsyncMeasure meas;
extern FindOptManager findOpt;
extern RemeasureManager remeasure;

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
extern float MAX_DIFF_TEMP;
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
extern volatile AppState currentAppState;
extern volatile BuildModelPhase buildModelPhase;
extern volatile IRState currentIRState;
extern float noiseFloorMv;
extern DisplayState currentDisplayState;
extern uint8_t overtemp_trip_counter;
extern unsigned long chargePhaseStartTime;
extern unsigned long chargingStartTime;
extern unsigned long lastChargeEvaluationTime;
extern const int pwmPin;
extern double THERMISTOR_1_OFFSET;


// --- Function Declarations ---

// from main .ino
void applyDuty(uint32_t duty);
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
void buildCurrentModel(bool warmStart);
float estimateCurrent(int dutyCycle);
int estimateDutyCycleForCurrent(float targetCurrent);
MeasurementData takeMeasurement(int dc, uint32_t stabilization_delay);
inline unsigned long unmanagedCastUL(unsigned long v){ return v; }

// from graphing.cpp
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
float estimateTempDiff(float voltageUnderLoad, float voltageNoLoad, float current, float internalResistanceParam, float ambientTempC, uint32_t currentTime, uint32_t lastChargeEvaluationTime, float BatteryTempC, float* unappliedEnergy_J = nullptr, float cellMassKg = DEFAULT_CELL_MASS_KG, float specificHeat = DEFAULT_SPECIFIC_HEAT, float area = DEFAULT_SURFACE_AREA_M2, float convectiveH = DEFAULT_CONVECTIVE_H, float emissivity = DEFAULT_EMISSIVITY);
void startRemeasure(float targetCurrent);
bool remeasureStep();


#endif // DEFINITIONS_H
