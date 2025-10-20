#ifndef CONFIG_H
#define CONFIG_H

// This file will be populated with hardware pin definitions and other constants.

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
#define PLOT_UPDATE_INTERVAL_MS 2000
#define PLOT_DATA_UPDATE_INTERVAL 1000
#define CHARGING_HOUSEKEEP_INTERVAL 150
#define IR_HANDLE_INTERVAL_MS 500
#define MAIN_VCC_RATIO 2.0
#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 75
#define PWM_FREQUENCY 1000
#define BUILD_CURRENT_MODEL_DELAY 200

// Physical defaults
static const float DEFAULT_CELL_MASS_KG       = 0.012f;
static const float DEFAULT_SPECIFIC_HEAT      = 1000.0f;
static const float DEFAULT_SURFACE_AREA_M2    = 0.001477f;
static const float DEFAULT_CONVECTIVE_H       = 0.2f;
static const float DEFAULT_EMISSIVITY         = 0.9f;
static const float STEFAN_BOLTZMANN           = 5.670374419e-8f;

// Charging constants
const float MAX_TEMP_DIFF_THRESHOLD = 0.5f;
const uint8_t OVERTEMP_TRIP_TRESHOLD = 3;
// extern float maximumCurrent; // This will be moved to DataStore
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
// extern float MAX_DIFF_TEMP; // This will be moved to DataStore
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 2.3f;
const float MIN_CURRENT = 0.0f;
const float MAX_CURRENT = 0.40f;

#endif // CONFIG_H