// Refactored version: simplest3_refactored_parted.ino
// Purpose: cosmetic and structural cleanup only — functionality left unchanged.
// Changes made:
//  - Normalized indentation (tabs -> 4 spaces)
//  - Removed trailing whitespace
//  - Limited consecutive blank lines to 2
//  - Added this header. No variable renames or logic changes were made.
// If you want a deeper refactor (extract functions, constants, remove globals,
// add non-blocking patterns, unit tests), tell me and I will proceed carefully.

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chips
#include <SPI.h>
#include <IRremote.h>
#include <cmath>     // For math functions like log, exp, pow, isnan
#include <limits>    // For std::numeric_limits
#include <vector>    // For dynamic arrays
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
#include "analog.h"
#include "graphing.h"
#include "logging.h"
#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
//#include <Fonts/FreeSans9pt7b.h> // Make sure you have included the font file
#define DEBUG_LABELS // uncomment to see debugging messages from graph plot label placement logic... rabbit hole warning!

// Define additional constants for charging (adjust as needed)
//#define STABILIZATION_DELAY_MS 100    // Delay after setting a duty cycle to allow voltage stabilization
#define CHARGING_UPDATE_INTERVAL_MS 2000  // Interval between re-evaluation during charging
#define PWM_MAX 255                   // Maximum PWM value (for analogWrite)
#define TEST_DUTY_CYCLE 128           // Initial test duty cycle value
//#define VOLTAGE_TOLERANCE 0.05        // Voltage tolerance (in volts) for convergence
#define TOTAL_TIMEOUT (20UL * 60 * 60 * 1000) // 20h total timeout after charge stops for safety reasons

// --- Timing intervals (ms)
#define PLOT_UPDATE_INTERVAL_MS     1000
#define CHARGING_HOUSEKEEP_INTERVAL 100
#define IR_HANDLE_INTERVAL_MS       200

// --- Timing trackers
unsigned long lastPlotUpdateTime     = 0;
unsigned long lastChargingHouseTime  = 0;
unsigned long lastIRHandleTime       = 0;


 // Constants for battery charging
const float MAX_TEMP_DIFF_THRESHOLD = 0.25f;         // Maximum temperature difference in Celsius before stopping charge
uint8_t overtemp_trip_counter = 0 ; // reset trip counter
const uint8_t OVERTEMP_TRIP_TRESHOLD = 3 ; // re check that many times . increases immunity against transient ambient temperature changes . 2*2 = 4minutes

// Physical defaults (tune to your cell / environment)
static const float DEFAULT_CELL_MASS_KG       = 0.015f;   // ~13 g (AAA NiMH ballpark)
static const float DEFAULT_SPECIFIC_HEAT      = 1000.0f;  // J/(kg·K) (order-of-magnitude; tune)
static const float DEFAULT_SURFACE_AREA_M2    = 0.001477f;// m^2 (10mm x 42mm cylinder approximation)
static const float DEFAULT_CONVECTIVE_H       = 4.4f;     // W/(m^2·K) free convection typical small cell
static const float DEFAULT_EMISSIVITY         = 0.9f;
static const float STEFAN_BOLTZMANN          = 5.670374419e-8f;

float maximumCurrent = 0.150;                         //maximum charging current 
const float MH_ELECTRODE_RATIO = 0.40f;               // Target ratio for MH electrode voltage
const uint32_t CHARGE_EVALUATION_INTERVAL_MS = 120000;   // Re-evaluate charging parameters every 2 minutes
// 600 points for 20 hours, so 19200 bytes data log . should fit into continous allocation of esp32 .. or not ...
const int CHARGE_CURRENT_STEP = 1;                  // Step size for PWM duty cycle adjustment
const int MAX_CHARGE_DUTY_CYCLE = 255;              // Maximum duty cycle for charging
const int MIN_CHARGE_DUTY_CYCLE = 5;                // Minimum duty cycle for charging
#define ISOLATION_THRESHOLD 0.02f // for oportunistic rint calculation - error distribution and averaging of data clusters

//end charge

// --- Configuration --- of R int measurement (and for current estimation)
//const int MAX_RESISTANCE_POINTS = 50; // Define as a constant
const float MEASURABLE_CURRENT_THRESHOLD = 0.005f; // Adjust as needed (40mA) - also used for estimation of current when duty cycle is below measurable current.
const int MIN_DUTY_CYCLE_START = 8;
const int MAX_DUTY_CYCLE = 255;
const int DUTY_CYCLE_INCREMENT_FIND_MIN = 5;
const int STABILIZATION_DELAY_MS = 2000;
const int STABILIZATION_PAIRS_FIND_DELAY_MS = 1000; // two task current measurement cycles minimum
const int UNLOADED_VOLTAGE_DELAY_MS = 5000;
const int MIN_DUTY_CYCLE_ADJUSTMENT_STEP = 5;
const float MIN_CURRENT_DIFFERENCE_FOR_PAIR = 0.02f;
const float MIN_VALID_RESISTANCE = 0.0f; // Threshold for considering a resistance value valid

// end of Rint configuration

// Structure to hold measurement data
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

// Define the structure to hold the current model (polynomial coefficients)
struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool isModelBuilt = false;
};

// Global instance of the current model
CurrentModel currentModel;

// Structure to hold MH electrode voltage measurement data
struct MHElectrodeData {
  float unloadedVoltage;
  float loadedVoltage;
 // float mhElectrodeRatio;
  float targetVoltage; // Added field for the target voltage
  float voltageDifference; // Added field for the difference between loaded and target voltage
  float current;
  uint32_t dutyCycle; // duty cycle
  uint32_t timestamp; // timestamp
};

enum ChargingState {
    CHARGE_IDLE = 0,
    CHARGE_FIND_OPT,   // Find optimal DC async
    CHARGE_MONITOR,    // Charging at applied duty; periodically re-evaluate
    CHARGE_STOPPED
};

ChargingState chargingState = CHARGE_IDLE;
unsigned long chargePhaseStartTime = 0;
int cachedOptimalDuty = MAX_CHARGE_DUTY_CYCLE;

enum FindPhase {
    FIND_IDLE,
    FIND_INIT_HIGHDC,     // we requested initial measure at high DC
    FIND_BINARY_PREPARE,  // ready to request a mid DC measurement
    FIND_BINARY_WAIT,     // waiting for measurement to complete
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
};

static FindOptManager findOpt;

enum MeasState {
    MEAS_IDLE,
    MEAS_STOPLOAD_WAIT,     // wrote duty=0, waiting unload delay
    MEAS_SAMPLE_UNLOADED,   // take unloaded sample (instant)
    MEAS_APPLY_LOAD,        // wrote test duty, waiting stabilization
    MEAS_SAMPLE_LOADED,     // take loaded sample (instant)
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
} meas; // single global measurement instance

// --- Configuration Section ---
// Define TFT display pins
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Define Thermistor pins
#define THERMISTOR_PIN_1 36 // Analog pin for Thermistor 1
#define THERMISTOR_PIN_1_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_PIN_1_OVERSAMPLING 16

//double THERMISTOR_1_OFFSET = -960.0;  // zero offset to cancel out slight thermistor differences
double THERMISTOR_1_OFFSET = 0.0;  // zero offset to cancel out slight thermistor differences and adc nonlinearity

#define THERMISTOR_VCC_PIN 35 // analog pin to measure VCC for thermistors
#define THERMISTOR_VCC_ATTENUATION ADC_ATTEN_DB_11
#define THERMISTOR_VCC_OVERSAMPLING 16

// Define Voltage Readout Pin
#define VOLTAGE_READ_PIN 39    // analog pin to measure battery voltage
#define VOLTAGE_ATTENUATION ADC_ATTEN_DB_11
#define VOLTAGE_OVERSAMPLING 16

//#define MAIN_VCC_RATIO 4.125 // multiplier to determine VCC from thermistor divider VCC
#define MAIN_VCC_RATIO 2.0 // multiplier to determine VCC from thermistor divider VCC

volatile uint32_t voltage_last_time;
volatile uint32_t voltage_update_interval = 250;

volatile float mAh_charged = 0.0f;
volatile bool resetAh = false; // Semaphore flag to reset mAh
volatile uint32_t mAh_last_time = 0; // Keep track of the last time mAh was updated

// Define Current Shunt Pin and Value
#define CURRENT_SHUNT_PIN 34
#define CURRENT_SHUNT_ATTENUATION ADC_ATTEN_DB_0
#define CURRENT_SHUNT_OVERSAMPLING 16

#define CURRENT_SHUNT_RESISTANCE 2.5f
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 75 // offset of current pin

// Define PWM Pin
#define PWM_PIN 19

// Fixed temperature ranges for scaling the plot - ADJUST IF NEEDED
const float MIN_TEMP             = 15.0;
const float MAX_TEMP             = 30.0;
const float MIN_DIFF_TEMP        = -0.5; // Difference range
float MAX_DIFF_TEMP        = 1.5;

// Fixed voltage range for scaling the voltage plot
const float MIN_VOLTAGE = 1.0f;
const float MAX_VOLTAGE = 2.3f;

// Fixed current range for scaling the current plot - ADJUST IF NEEDED
const float MIN_CURRENT = 0.0f;
const float MAX_CURRENT = 0.40f; // Adjust this based on your expected current range

// Plotting parameters - Maximized graph and bottom labels - ADJUST IF NEEDED
#define PLOT_WIDTH          320           // Maximize width (almost full screen)
#define PLOT_HEIGHT         (216 - 3)     // Maximize height (leaving space for labels at bottom)
#define PLOT_X_START        0             // Adjusted start position for wider graph
#define PLOT_Y_START        0             // Adjusted start position from top

// Plot colors - ADJUST IF DESIRED
#define PLOT_X_AXIS_COLOR   TFT_WHITE
#define PLOT_Y_AXIS_COLOR   TFT_WHITE
#define PLOT_ZERO_COLOR     0x62ec

#define GRAPH_COLOR_1       TFT_RED
#define GRAPH_COLOR_2       TFT_GREEN
#define GRAPH_COLOR_DIFF    TFT_BLUE
#define GRAPH_COLOR_VOLTAGE TFT_YELLOW
#define GRAPH_COLOR_CURRENT TFT_MAGENTA // Define a color for the current plot
#define GRAPH_COLOR_RESISTANCE TFT_ORANGE
#define GRAPH_COLOR_RESISTANCE_PAIR TFT_CYAN

// Label area at the bottom
#define LABEL_Y_START       PLOT_Y_START + PLOT_HEIGHT + 3 // Start labels below graph
#define LABEL_TEXT_SIZE     1

// IR Remote setup
#define IR_RECEIVE_PIN 15

// IR Remote Key Definitions
namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0xE6,

    KEY_0 = 0x11,
    KEY_1 = 0x04,
    KEY_2 = 0x05,
    KEY_3 = 0x06,
    KEY_4 = 0x08,
    KEY_5 = 0x09,
    KEY_6 = 0x0A,
    KEY_7 = 0x0C,
    KEY_8 = 0x0D,
    KEY_9 = 0x0E,
    KEY_UP          = 0x60,
    KEY_DOWN        = 0x61,
    KEY_LEFT        = 0x65,
    KEY_RIGHT       = 0x62,
    KEY_OK          = 0x68,
    KEY_MENU        = 0x79,

    KEY_RED         = 0x6c,
    KEY_GREEN       = 0x14,
    KEY_YELLOW      = 0x15,
    KEY_BLUE        = 0x16,

    KEY_VOL_UP      = 0x07,
    KEY_VOL_DOWN    = 0x0b,
    KEY_CH_UP       = 0x12,
    KEY_CH_DOWN     = 0x10,

    KEY_REWIND      = 0x45,
    KEY_PLAY        = 0x47,
    KEY_PAUSE       = 0x4A,
    KEY_FORWARD     = 0x48,
    KEY_STOP        = 0x46,

    KEY_SETTINGS    = 0x1A,
    KEY_INFO        = 0x1F,
    KEY_SUBTITLES   = 0x25,
    KEY_MUTE        = 0x0F,
    KEY_NETFLIX     = 0xF3,
    KEY_PRIME_VIDEO = 0xF4,
    KEY_GUIDE       = 0x4F,
    KEY_SOURCE      = 0x01

  };
}

// --- Global variables ---

// Global variables for charging state
bool isCharging = false;
unsigned long lastChargeEvaluationTime = 0;
float mhElectrodeVoltage = 0.0f;
int chargingDutyCycle = 0;



TFT_eSPI tft = TFT_eSPI(); // TFT_eSPI instance

// --- Temperature sensor objects
SHT4xSensor sht4Sensor;
ThermistorSensor thermistorSensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

// Temperature reading arrays for plotting
float temp1_values[PLOT_WIDTH];
float temp2_values[PLOT_WIDTH];
float diff_values[PLOT_WIDTH];

// Voltage reading array for plotting
float voltage_values[PLOT_WIDTH];

// Current reading array for plotting
float current_values[PLOT_WIDTH];

// Global variable for voltage readout
volatile float voltage_mv = 1000.0f;

// Global variable for current readout
volatile float current_ma = 0.0f;

// --- PWM Variables ---
#define PWM_FREQUENCY 1000 // define frequency
const int pwmPin = PWM_PIN;
const int pwmResolutionBits = 8; // You can adjust the resolution (e.g., 8, 10, 12)
const int pwmMaxDutyCycle = (1 << pwmResolutionBits) - 1;
unsigned long pwmStartTime = 0;
uint32_t dutyCycle = 0 ;

// --- Internal Resistance Measurement Variables ---
const int MAX_RESISTANCE_POINTS = 100; // Maximum number of data points for internal resistance
float internalResistanceData[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance]
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2]; // [current, internal_resistance] // from consecutive pairs
int resistanceDataCountPairs = 0;
float regressedInternalResistance = 0 ;
float regressedInternalResistancePairs = 0 ; // from consecutive pairs
// Global variables to store the results of linear regression
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;

float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

bool isMeasuringResistance = false;

// --- Function Declarations ---
void setupPWM();
void measureInternalResistance();
void processThermistorData(const MeasurementData& data, const String& measurementType );
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
MHElectrodeData measureMHElectrodeVoltage(int testDutyCycle);
int findOptimalChargingDutyCycle(int maxChargeDutyCycle, int suggestedStartDutyCycle);
void handleIRCommand();
MeasurementData takeMeasurement(int dc,uint32_t stabilization_delay);


// --- Function Implementations ---

#define BUILD_CURRENT_MODEL_DELAY 200 // 200ms is enough (current measurement takes about 100ms)

// duty cycle to current modelling function
// Function to build the current model
void buildCurrentModel(bool warmStart = false) {
    Serial.println("Building current model...");
    std::vector<float> dutyCycles;
    std::vector<float> currents;

    // Warm start: Load previous data if available (you'd need to implement storage)
    if (warmStart && currentModel.isModelBuilt) {
        Serial.println("Warm start: Accumulating data with previous model.");
        // For simplicity, we'll just continue adding new data in this example.
        // In a real application, you might load previously saved dutyCycles and currents.
    } else {
        Serial.println("Starting fresh model building.");
        dutyCycles.clear();
        currents.clear();
        // Ensure the first point is (0, 0)
        dutyCycles.push_back(0.0f);
        currents.push_back(0.0f);
    }

    // Sweep through duty cycles and measure current
    for (int dutyCycle = 1; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle += 5) { // Adjust step for faster sweep
        MeasurementData data = takeMeasurement(dutyCycle, BUILD_CURRENT_MODEL_DELAY);
        processThermistorData(data, "Estimating Min Current");
        if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
            dutyCycles.push_back(static_cast<float>(dutyCycle));
            currents.push_back(data.current);
        } else {
            Serial.printf("Current below threshold (%.3f A) at duty cycle %d. Skipping.\n", data.current, dutyCycle);
        }
    }

    if (dutyCycles.size() < 2) {
        Serial.println("Not enough data points to build a reliable model.");
        currentModel.isModelBuilt = false;
        dutyCycle = 0 ;
        analogWrite(pwmPin, dutyCycle);
        return;
    }

    // Polynomial regression using Eigen
    int degree = 3; // Choose the degree of the polynomial
    int numPoints = dutyCycles.size();
    Eigen::MatrixXd A(numPoints, degree + 1);
    Eigen::VectorXd b(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = std::pow(dutyCycles[i], j);
        }
        b(i) = currents[i];
    }

    // Solve for the coefficients using least squares
    Eigen::VectorXd coefficients = A.householderQr().solve(b);

    // Ensure the model passes through (0, 0) by forcing the constant term to zero
    if (degree >= 0) {
        coefficients(0) = 0.0f;
    }

    currentModel.coefficients = coefficients;
    currentModel.isModelBuilt = true;

    Serial.println("Current model built successfully with coefficients:");
    for (int i = 0; i <= degree; ++i) {
        Serial.printf("Coefficient for x^%d: %.4f\n", i, coefficients(i));
    }
    dutyCycle = 0 ;
    analogWrite(pwmPin, dutyCycle);

}

// Helper function to estimate current based on duty cycle
float estimateCurrent(int dutyCycle) {
    if (!currentModel.isModelBuilt) {
        Serial.println("Warning: Current model has not been built yet. Returning 0.");
        return 0.0f;
    }

    float estimatedCurrent = 0.0f;
    float dutyCycleFloat = static_cast<float>(dutyCycle);
    for (int i = 0; i < currentModel.coefficients.size(); ++i) {
        estimatedCurrent += currentModel.coefficients(i) * std::pow(dutyCycleFloat, i);
    }

    if (estimatedCurrent < MEASURABLE_CURRENT_THRESHOLD && dutyCycle > 0 ) {
        Serial.printf("Estimated current (%.3f A) below threshold at duty cycle %d. Inferring.\n", estimatedCurrent, dutyCycle);
    }

    return std::max(0.0f, estimatedCurrent); // Ensure current is not negative
}

// end duty cycle to current modeling function

// global variables for sht4x readouts (now managed by the class)
// float aTemperature = 25.0;
// float aHumidity = 50.0;

void task_readSHT4x(void* parameter) {
    while (true) {
        sht4Sensor.read();
        vTaskDelay(100);
    }
}

// Modified task_readThermistor function
void task_readThermistor(void* parameter) {
    mAh_last_time = millis(); // Initialize the last update time for mAh

    while (true) {
        uint32_t current_time = millis();

        // Wait for SHT4 sensor data if locked
        while (sht4Sensor.isLocked()) {
            vTaskDelay(10 / portTICK_PERIOD_MS); // Use portTICK_PERIOD_MS for FreeRTOS delays
        };

        // Read thermistor data (assuming this also updates VCC)
        thermistorSensor.read(sht4Sensor.getTemperature());

        // Read current and store in global variable (for model building)
        int task_current_numSamples = 256;
        double sumAnalogValuesCurrent = 0;
        for (int i = 0; i < task_current_numSamples; ++i) {
            //uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN); // Assuming your analogReadMillivolts function handles attenuation and oversampling
            uint32_t analogValue = analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, CURRENT_SHUNT_OVERSAMPLING);
            sumAnalogValuesCurrent += analogValue;
        }
        double voltageAcrossShunt = (sumAnalogValuesCurrent / task_current_numSamples) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
        current_ma = (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE); // in mA

        // Read voltage periodically
        if ((current_time - voltage_last_time) > voltage_update_interval) {
            int task_voltage_numSamples = 256;
            double sumAnalogValuesVoltage = 0;
            for (int i = 0; i < task_voltage_numSamples; ++i) {
                //uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN); // Assuming your analogReadMillivolts function handles attenuation and oversampling
                uint32_t analogValue = analogReadMillivolts(VOLTAGE_READ_PIN, VOLTAGE_ATTENUATION, VOLTAGE_OVERSAMPLING);
                sumAnalogValuesVoltage += analogValue;
            }
            // Assuming thermistorSensor.getVCC() provides the ESP32 VCC in mV
            // Adjust MAIN_VCC_RATIO based on your voltage divider if needed
            // The line below assumes you are measuring the battery voltage directly (after any potential divider)
            voltage_mv = (thermistorSensor.getVCC() * MAIN_VCC_RATIO) - (sumAnalogValuesVoltage / task_voltage_numSamples);
            voltage_last_time = current_time;
        }

        // Measure mAh
        uint32_t time_elapsed = current_time - mAh_last_time; // Time elapsed in milliseconds
        double time_elapsed_hours = (double)time_elapsed / (1000.0 * 3600.0);
        double current_for_mah_calculation_ma;

        if (currentModel.isModelBuilt&& (current_ma/1000.0)<MEASURABLE_CURRENT_THRESHOLD) {
            // Use the estimated current for mAh calculation
            current_for_mah_calculation_ma = static_cast<double>(estimateCurrent(dutyCycle)*1000.0); // Convert Amps to mA
        } else {
            // Use the measured current for mAh calculation
            current_for_mah_calculation_ma = current_ma;
        }

        // Calculate mAh charged during this interval
        mAh_charged += (current_for_mah_calculation_ma) * time_elapsed_hours; // Convert mA to A, then to mAh

        mAh_last_time = current_time;

        // Check and reset mAh if the flag is set
        if (resetAh) {
            mAh_charged = 0.0f;
            resetAh = false; // Clear the flag after resetting
            Serial.println("mAh counter reset.");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Use portTICK_PERIOD_MS for FreeRTOS delays
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize TFT
    tft.init();
    tft.setRotation(1); // Adjust rotation as needed
    // Clear screen and set background color
    tft.fillScreen(TFT_BLACK);
    //tft.loadFont(FreeSans9pt7b);
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);

    // Initialize temperature, voltage, and current arrays to a default value
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f; // Initialize voltage array
        current_values[i] = 0.0f;     // Initialize current array
    }

    sht4Sensor.begin(); // Initialize SHT4x sensor
    thermistorSensor.begin(); // Initialize thermistor sensor

//  analogSetAttenuation(ADC_0db);

    // setting of per pin attentuation is broken in the IDF...

    // analogSetPinAttenuation(VOLTAGE_READ_PIN, ADC_11db) ; // Set attenuation to 11dB
    //analogSetPinAttenuation(CURRENT_SHUNT_PIN, ADC_0db); // Set attenuation for current shunt pin
    //analogSetPinAttenuation(THERMISTOR_PIN_1, ADC_0db);
    //analogSetPinAttenuation(thermistor2Pin, ADC_0db);
    //analogSetPinAttenuation(THERMISTOR_VCC_PIN, ADC_0db);

    xTaskCreate(task_readSHT4x,     "SHT4",  4096, NULL, 1, NULL);
    xTaskCreate(task_readThermistor, "THERM", 4096, NULL, 1, NULL); // Create the new thermistor reading task

    setupPWM(); // Initialize PWM

    Serial.println("Ni-Cd/Ni-MH battery charger. use samsung DVD remote to control");
    Serial.println("PLAY to measure internal resistance of battery");
    Serial.println("INFO to see internal resistance graph");
    Serial.println("POWER to start charging");
    Serial.println("SOURCE to see charge graph");
#ifdef DEBUG_LABELS
    Serial.println("0 for charge graph test and label placement test");
#endif //#ifdef DEBUG_LABELS

    // Start internal resistance measurement at startup for demonstration
   // isMeasuringResistance = true;
}

void setupPWM() {
    analogWriteResolution(pwmPin,pwmResolutionBits);
    analogWriteFrequency(pwmPin,PWM_FREQUENCY);
    pinMode(pwmPin, OUTPUT);
    pwmStartTime = millis(); // Initialize the start time for the PWM cycle
    dutyCycle = 0;
    analogWrite(pwmPin, 0); // Initialize PWM with 0 duty cycle
}

// Bubble Sort implementation
void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                // Swap data[j] and data[j+1]
                float temp0 = data[j][0];
                float temp1 = data[j][1];
                data[j][0] = data[j + 1][0];
                data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0;
                data[j + 1][1] = temp1;
            }
        }
    }
}

// ---------------measure Rint

// Function to take a measurement
MeasurementData takeMeasurement(int dc,uint32_t stabilization_delay) {
    dutyCycle = dc;
    analogWrite(pwmPin, dc);
//    delay(STABILIZATION_DELAY_MS);
    delay(stabilization_delay);
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = dc;
    data.timestamp = millis();
    return data;
}

// Function to stop the load (duty cycle 0) and wait
void stopLoad() {
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    delay(UNLOADED_VOLTAGE_DELAY_MS);
}

// Function to get an unloaded voltage measurement
MeasurementData getUnloadedVoltageMeasurement() {
    stopLoad();
    MeasurementData data;
    getThermistorReadings(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    data.dutyCycle = 0;
    data.timestamp = millis();
    return data;
}

// Function to process and store resistance data
void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count < MAX_RESISTANCE_POINTS && resistance > MIN_VALID_RESISTANCE) {
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

// Function to print and update thermistor data on serial and TFT
void processThermistorData(const MeasurementData& data, const String& measurementType ) {
    printThermistorSerial(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    updateTemperatureHistory(data.temp1, data.temp2, data.tempDiff, data.voltage, data.current);
    prepareTemperaturePlot();
    plotVoltageData();
    plotTemperatureData();
    displayTemperatureLabels(data.temp1, data.temp2, data.tempDiff, data.t1_millivolts, data.voltage, data.current);
    bigUglyMessage(measurementType);
}

// Function to clear a specific area for updating text
void clearTextArea(int y_start, int height) {
  tft.fillRect(0, y_start, 320, height, TFT_BLACK);
}

// Step 1: Find the minimal duty cycle for measurable current using binary search
int findMinimalDutyCycle() {
  Serial.println("Finding minimal duty cycle for measurable current using binary search...");
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.drawString("Finding Min Duty Cycle", 10, 10);

  int low = MIN_DUTY_CYCLE_START;
  int high = MAX_DUTY_CYCLE;
  int minimalDutyCycle = 0; // Initialize to indicate not found yet
  int iteration = 0;

  // --- New variables for the graph ---
  const int GRAPH_X = 30;
  const int GRAPH_Y = 50; // Adjust Y position as needed below the search bar
  const int GRAPH_WIDTH = SCREEN_WIDTH - 2;
  const int GRAPH_HEIGHT = SCREEN_HEIGHT-90;
  const int MAX_GRAPH_POINTS = 100; // Maximum number of points to store
  int dutyCyclePoints[MAX_GRAPH_POINTS];
  float currentPoints[MAX_GRAPH_POINTS];
  int numPoints = 0;
  float maxCurrent = 0.0; // To scale the Y-axis of the graph
  // --- End of new variables ---

  while (low <= high) {
    iteration++;
    int mid = low + (high - low) / 2; // Calculate middle duty cycle

    Serial.printf("Iteration: %d, Testing duty cycle: %d\n", iteration, mid);
    MeasurementData data = takeMeasurement(mid, STABILIZATION_DELAY_MS);
    Serial.printf("Measured current at %d%% duty cycle: %.3f A\n", mid, data.current);
    stopLoad(); // Stop load after each measurement

    // Clear the area for the duty cycle bar and text
    tft.fillRect(0, DUTY_CYCLE_BAR_Y , SCREEN_WIDTH-DUTY_CYCLE_BAR_START_X, SCREEN_HEIGHT - DUTY_CYCLE_BAR_Y, BLACK);

    // Draw the duty cycle bar and information
    drawDutyCycleBar(low, high, mid, data.current, MEASURABLE_CURRENT_THRESHOLD);

    // --- Store data for the graph ---
    if (numPoints < MAX_GRAPH_POINTS) {
      dutyCyclePoints[numPoints] = mid;
      currentPoints[numPoints] = data.current;
      numPoints++;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      }
    } else {
      // If we reach the maximum number of points, shift the array to make space for the new point
      for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
        dutyCyclePoints[i] = dutyCyclePoints[i + 1];
        currentPoints[i] = currentPoints[i + 1];
      }
      dutyCyclePoints[MAX_GRAPH_POINTS - 1] = mid;
      currentPoints[MAX_GRAPH_POINTS - 1] = data.current;
      if (data.current > maxCurrent) {
        maxCurrent = data.current;
      } else {
        // Recalculate maxCurrent if the newest point is not the maximum
        maxCurrent = 0.0;
        for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
          if (currentPoints[i] > maxCurrent) {
            maxCurrent = currentPoints[i];
          }
        }
      }
    }
    // --- End of storing data ---

    // --- Draw the graph ---
    drawGraph(dutyCyclePoints, currentPoints, numPoints, maxCurrent, GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT);
    // --- End of drawing the graph ---

    if (data.current >= MEASURABLE_CURRENT_THRESHOLD) {
      // Found a duty cycle with measurable current.
      // It might be the minimal, so store it and try lower values.
      minimalDutyCycle = mid;
      high = mid - 1; // Search in the lower half
      Serial.printf("Found measurable current at %d%%, trying lower values.\n", mid);
    } else {
      // Current is below the threshold, need a higher duty cycle.
      low = mid + 1; // Search in the upper half
      Serial.printf("Current too low at %d%%, trying higher values.\n", mid);
    }
  }

  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);

  if (minimalDutyCycle > 0) {
    Serial.printf("Minimal duty cycle found: %d\n", minimalDutyCycle);
    tft.printf("Minimal Duty Cycle Found:\n%d%%\n", minimalDutyCycle);
    return minimalDutyCycle;
  } else {
    Serial.println("Warning: Could not find a duty cycle producing measurable current.");
    tft.println("Warning: Could not find\nmeasurable current.");
    return 0; // Indicate failure
  }
}


// Improved function to generate duty cycle pairs with approximately linear spacing
// across highDc current increments using binary search
std::vector<std::pair<int, int>> generateDutyCyclePairs(int minDutyCycle) {
    Serial.println("Generating duty cycle pairs (improved for linear current spacing using binary search)...");
    std::vector<std::pair<int, int>> pairs;
    if (minDutyCycle == 0) {
        return pairs; // Return empty if no min duty cycle found
    }

    int numPairs = MAX_RESISTANCE_POINTS / 2;
    if (numPairs < 1) {
        Serial.println("MAX_RESISTANCE_POINTS should be at least 2 for paired measurement.");
        return pairs;
    }

    // First, let's estimate the current range by measuring at min and max duty cycles
    MeasurementData minCurrentData = takeMeasurement(minDutyCycle, STABILIZATION_DELAY_MS);
    processThermistorData(minCurrentData, "Estimating Min Current");
    float minCurrent = minCurrentData.current;

    MeasurementData maxCurrentData = takeMeasurement(MAX_DUTY_CYCLE, STABILIZATION_DELAY_MS);
    processThermistorData(maxCurrentData, "Estimating Max Current");
    float maxCurrent = maxCurrentData.current;

    if (maxCurrent <= minCurrent) {
        Serial.println("Warning: Maximum current is not greater than minimum current. Cannot ensure linear spacing. Falling back to a simpler approach.");
        // Fallback to a simpler linear duty cycle spacing (modified from original)
        int highDc = MAX_DUTY_CYCLE;
        int lowDc = minDutyCycle;
        int dutyCycleStep = (MAX_DUTY_CYCLE - minDutyCycle) / numPairs;
        for (int i = 0; i < numPairs; ++i) {
            if (highDc < lowDc) break;
            MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
            processThermistorData(lowData, "Generating Pairs (Fallback)");
            // if (tft) {
            //     tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            //     tft.printf("progress: %.1f ", ((float)i / numPairs) * 100.0);
            // }
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < highDc) {
                int adjustedLowDc = lowDc;
                for (int j = 0; j < 10; ++j) {
                    adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                    if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                    MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                    if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                        lowDc = adjustedLowDc;
                        Serial.printf("Adjusting low duty cycle to %d due to low current (fallback).\n", lowDc);
                        break;
                    }
                }
                if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                    Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current (fallback).");
                }
            } else if (lowDc > MAX_DUTY_CYCLE) {
                Serial.println("Warning: Low duty cycle exceeded maximum value (fallback).");
                break;
            }
            pairs.push_back({lowDc, highDc});
            highDc -= dutyCycleStep;
            lowDc += dutyCycleStep; // Increment lowDc as well for a more even spread
            if (highDc < minDutyCycle) highDc = minDutyCycle + 1;
            if (lowDc > MAX_DUTY_CYCLE) lowDc = MAX_DUTY_CYCLE - 1;
        }
        Serial.printf("Generated %zu duty cycle pairs (fallback).\n", pairs.size());
        return pairs;
    }

    float totalCurrentRange = maxCurrent - minCurrent;
    float desiredCurrentIncrement = totalCurrentRange / numPairs;

    int lowDc = minDutyCycle;
    int previousHighDc = MAX_DUTY_CYCLE;

    for (int i = 0; i < numPairs; ++i) {
        float targetHighCurrent = maxCurrent - (i * desiredCurrentIncrement);
        int bestHighDc = -1;
        float minCurrentDifference = 1e9; // Initialize with a large value

        int lowBound = minDutyCycle;
        int highBound = previousHighDc;

        while (lowBound <= highBound) {
            int midDc = lowBound + (highBound - lowBound) / 2;
            MeasurementData midData = takeMeasurement(midDc, STABILIZATION_PAIRS_FIND_DELAY_MS);
            processThermistorData(midData, "Binary Searching High DC");
            float currentDifference = std::fabs(midData.current - targetHighCurrent);

            if (currentDifference < minCurrentDifference) {
                minCurrentDifference = currentDifference;
                bestHighDc = midDc;
            }

            if (midData.current > targetHighCurrent) {
                highBound = midDc - 1; // Search in the lower half
            } else {
                lowBound = midDc + 1; // Search in the upper half
            }
        }

        int currentHighDc = bestHighDc != -1 ? bestHighDc : previousHighDc;
        if (currentHighDc < minDutyCycle) currentHighDc = minDutyCycle;

        // Adjust lowDc (same logic as before)
        MeasurementData lowData = takeMeasurement(lowDc, STABILIZATION_DELAY_MS);
        processThermistorData(lowData, "Generating Pairs");
             tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
             tft.printf("progress: %.0f ", ((float)i / numPairs) * 100.0);

        if (lowData.current < MEASURABLE_CURRENT_THRESHOLD && lowDc < currentHighDc) {
            int adjustedLowDc = lowDc;
            for (int j = 0; j < 5; ++j) {
                adjustedLowDc += MIN_DUTY_CYCLE_ADJUSTMENT_STEP;
                if (adjustedLowDc > MAX_DUTY_CYCLE) break;
                MeasurementData checkData = takeMeasurement(adjustedLowDc, STABILIZATION_DELAY_MS);
                if (checkData.current >= MEASURABLE_CURRENT_THRESHOLD) {
                    lowDc = adjustedLowDc;
                    Serial.printf("Adjusting low duty cycle to %d due to low current.\n", lowDc);
                    break;
                }
            }
            if (lowData.current < MEASURABLE_CURRENT_THRESHOLD) {
                Serial.println("Warning: Could not adjust low duty cycle to achieve measurable current.");
                //break; // Consider if breaking here is appropriate
            }
        } else if (lowDc > MAX_DUTY_CYCLE) {
            Serial.println("Warning: Low duty cycle exceeded maximum value.");
            break;
        }

        if (currentHighDc < lowDc) {
            currentHighDc = lowDc + 1;
            break;
        }; // Ensure highDc is not lower than lowDc

        pairs.push_back({lowDc, currentHighDc});
        previousHighDc = currentHighDc - 1; // Start the next search from slightly below the current highDc

        // Optional: Print debug information
        // MeasurementData actualHighData = takeMeasurement(currentHighDc,STABILIZATION_DELAY_MS);
        // Serial.printf("Pair %d: lowDc=%d, highDc=%d, targetCurrent=%.2f, actualCurrent=%.2f\n",
        //                 i, lowDc, currentHighDc, targetHighCurrent, actualHighData.current);

        if (previousHighDc < minDutyCycle) break;
    }

    Serial.printf("Generated %zu duty cycle pairs (for linear current spacing using binary search).\n", pairs.size());
    return pairs;
}

// Step 3: Measure internal resistance using loaded/unloaded method
void measureInternalResistanceLoadedUnloaded(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& voltagesLoaded, std::vector<float>& currentsLoaded, std::vector<float>& dutyCycles) {
    Serial.println("\n--- Measuring Internal Resistance (Loaded/Unloaded) ---");
    for (const auto& pair : dutyCyclePairs) {
        int dc = pair.second; // Use the high duty cycle for loaded measurement here
        Serial.printf("--- Duty Cycle (Loaded/Unloaded): %d ---\n", dc);

        // Measure voltage with duty cycle applied (loaded)
        MeasurementData loadedData = takeMeasurement(dc,STABILIZATION_DELAY_MS);
        processThermistorData(loadedData, "Rint L/UL");

        voltagesLoaded.push_back(loadedData.voltage);
        currentsLoaded.push_back(loadedData.current);
        dutyCycles.push_back(static_cast<float>(dc));

        Serial.printf("Duty Cycle ON (%d): Voltage: %.3f V, Current: %.3f A\n", dc, loadedData.voltage, loadedData.current);

        // Measure voltage with duty cycle off (unloaded) immediately after
        MeasurementData unloadedData = getUnloadedVoltageMeasurement();
        Serial.printf("Duty Cycle OFF: Voltage: %.3f V, Current: %.3f A\n", unloadedData.voltage, unloadedData.current);

        // Calculate internal resistance for this step using the immediately following unloaded voltage
        if (loadedData.current > 0.01f) {
            float internalResistance = (unloadedData.voltage - loadedData.voltage) / loadedData.current;
            storeResistanceData(loadedData.current, std::fabs(internalResistance), internalResistanceData, resistanceDataCount);
            Serial.printf("Calculated Internal Resistance (Loaded-Unloaded): %.3f Ohm\n", std::fabs(internalResistance));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 30);
            tft.printf("(L/UL): %.3f ", std::fabs(internalResistance));
        } else {
            Serial.println("Warning: Current is too low to reliably calculate internal resistance (Loaded-Unloaded).");
            storeResistanceData(loadedData.current, -1.0f, internalResistanceData, resistanceDataCount); // Indicate invalid
        }
        Serial.println("---------------------------\n");
    }
}

// Step 4: Measure internal resistance using duty cycle pairs
void measureInternalResistancePairs(const std::vector<std::pair<int, int>>& dutyCyclePairs, std::vector<float>& consecutiveInternalResistances) {
    Serial.println("\n--- Measuring Internal Resistance using Duty Cycle Pairs ---");
    for (const auto& pair : dutyCyclePairs) {
        int dcLow = pair.first;
        int dcHigh = pair.second;

        Serial.printf("--- Duty Cycle Pair: Low=%d, High=%d ---\n", dcLow, dcHigh);

        // Measure voltage and current at low duty cycle
        MeasurementData lowData = takeMeasurement(dcLow,STABILIZATION_DELAY_MS);
        Serial.printf("Duty Cycle Low (%d): Voltage: %.3f V, Current: %.3f A\n", dcLow, lowData.voltage, lowData.current);

        // Measure voltage and current at high duty cycle
        MeasurementData highData = takeMeasurement(dcHigh,STABILIZATION_DELAY_MS);
        processThermistorData(highData, "Rint Pair");
        Serial.printf("Duty Cycle High (%d): Voltage: %.3f V, Current: %.3f A\n", dcHigh, highData.voltage, highData.current);

        // Calculate internal resistance using the pair
        if (highData.current > lowData.current + MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
            float internalResistanceConsecutive = (lowData.voltage - highData.voltage) / (highData.current - lowData.current);
            consecutiveInternalResistances.push_back(std::fabs(internalResistanceConsecutive));
            storeResistanceData(highData.current, std::fabs(internalResistanceConsecutive), internalResistanceDataPairs, resistanceDataCountPairs);
            Serial.printf("Calculated Internal Resistance (Pair): %.3f Ohm\n", std::fabs(internalResistanceConsecutive));
            tft.setCursor(PLOT_X_START + 5, PLOT_Y_START + PLOT_HEIGHT / 2 - 50);
            tft.printf("(Pair): %.3f ", std::fabs(internalResistanceConsecutive));
        } else {
            Serial.println("Warning: Current difference is too small to reliably calculate internal resistance (Pair).");
            consecutiveInternalResistances.push_back(-1.0f); // Indicate invalid
            storeResistanceData(highData.current, -1.0f, internalResistanceDataPairs, resistanceDataCountPairs);
        }
        Serial.println("---------------------------\n");
    }
}

// Step 5: Calculate average internal resistance from pairs
float calculateAverageInternalResistance(const std::vector<float>& resistances) {
    float sum = 0;
    int count = 0;
    for (float rint : resistances) {
        if (rint > MIN_VALID_RESISTANCE) {
            sum += rint;
            count++;
        }
    }
    if (count > 0) {
        float average = sum / count;
        Serial.printf("\nAverage Internal Resistance (Pairs): %.3f Ohm (%d valid points)\n", average, count);
        return average;
    } else {
        Serial.println("\nNo valid internal resistance measurements from pairs.");
        return -1.0f;
    }
}

// Step 6: Perform linear regression on loaded data
void performLinearRegression(const std::vector<float>& voltages, const std::vector<float>& currents) {
    if (voltages.size() >= 2) {
        Serial.println("Calculating overall internal resistance using linear regression (Loaded Data)...");

        double sumI = std::accumulate(currents.begin(), currents.end(), 0.0);
        double sumV = std::accumulate(voltages.begin(), voltages.end(), 0.0);
        double sumII = std::inner_product(currents.begin(), currents.end(), currents.begin(), 0.0);
        double sumIV = 0.0;
        for (size_t i = 0; i < currents.size(); ++i) {
            sumIV += currents[i] * voltages[i];
        }

        int n = voltages.size();
        double denominator = (n * sumII - sumI * sumI);

        if (std::fabs(denominator) > 1e-9) {
            double overallInternalResistance = (n * sumIV - sumI * sumV) / denominator;
            double openCircuitVoltage = (sumV - overallInternalResistance * sumI) / n;

            regressedInternalResistance = overallInternalResistance;
            Serial.printf("Calculated Overall Internal Resistance (Linear Regression on Loaded Data): %.3f Ohm\n", std::fabs(overallInternalResistance));
            Serial.printf("Estimated Open Circuit Voltage: %.3f V\n", openCircuitVoltage);
        } else {
            Serial.println("Error: Could not calculate overall internal resistance (division by zero in regression).");
        }
    } else {
        Serial.println("Not enough data points to perform overall linear regression on loaded data.");
    }
}

// ---------------measure Rint function itself

void measureInternalResistance() {
    if (!isMeasuringResistance) return;

    Serial.println("Starting improved internal resistance measurement...");

    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;

    std::vector<float> voltagesLoaded;
    std::vector<float> currentsLoaded;
    std::vector<float> dutyCycles;
    std::vector<float> consecutiveInternalResistances;
    std::vector<float> unloadedVoltagesHistory;
    std::vector<unsigned long> unloadedVoltageTimestamps;

    // Measure initial unloaded voltage
    MeasurementData initialUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(initialUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(initialUnloaded.timestamp);
    Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

    // Find minimal duty cycle
    int minDutyCycle = findMinimalDutyCycle();
    if (minDutyCycle == 0) {
        return;
    }

    // Generate duty cycle pairs
    std::vector<std::pair<int, int>> dutyCyclePairs = generateDutyCyclePairs(minDutyCycle);

    // Measure internal resistance using loaded/unloaded method
    measureInternalResistanceLoadedUnloaded(dutyCyclePairs, voltagesLoaded, currentsLoaded, dutyCycles);

    // Measure internal resistance using duty cycle pairs
    measureInternalResistancePairs(dutyCyclePairs, consecutiveInternalResistances);

    // Measure final unloaded voltage
    MeasurementData finalUnloaded = getUnloadedVoltageMeasurement();
    unloadedVoltagesHistory.push_back(finalUnloaded.voltage);
    unloadedVoltageTimestamps.push_back(finalUnloaded.timestamp);
    Serial.printf("Final Unloaded Voltage: %.3f V\n", finalUnloaded.voltage);

    // Sort the collected data points by current
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    // Calculate average internal resistance from consecutive steps (pairs)
//    calculateAverageInternalResistance(consecutiveInternalResistances);

    // Optional: Perform Linear Regression on the loaded voltage and current data
//    performLinearRegression(voltagesLoaded, currentsLoaded);

    // Perform linear regression after finding the optimal duty cycle
    if (resistanceDataCount >= 2) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
    }

    if (resistanceDataCountPairs >= 2) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of paired resistance.");
    }

    // Measurement complete
    Serial.printf("Internal resistance measurement complete. %d loaded/unloaded points, %d pair points collected.\n", resistanceDataCount, resistanceDataCountPairs);
    isMeasuringResistance = false; // Only measure once (adjust logic as needed)
}

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    while (thermistorSensor.isLocked()) {
        yield();
    } // Wait for thermistor task to update data
    temp1 = sht4Sensor.getTemperature(); // Thermistor 1 (Top) now comes from SHT4x
    temp2 = thermistorSensor.getTemperature2();
    tempDiff = thermistorSensor.getDifference();
    t1_millivolts = thermistorSensor.getRawMillivolts1();
    voltage = voltage_mv / 1000.0f; // Convert mV to Volts
    current = current_ma / 1000.0f; // Convert mA to Amps
}

// charge
//      processThermistorData(minCurrentData, "CHARGING");

// Helper function to perform binary search to find the insertion point or closest element
int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    int low = 0;
    int high = count - 1;
    int mid;

    while (low <= high) {
        mid = low + (high - low) / 2;
        if (data[mid][0] == targetCurrent) {
            return mid;
        } else if (data[mid][0] < targetCurrent) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    // If not found, return the index where it should be inserted or the closest index
    if (low < count) {
        if (high >= 0 && (targetCurrent - data[high][0] < data[low][0] - targetCurrent)) {
            return high;
        } else {
            return low;
        }
    } else {
        return high;
    }
}

// Helper function to insert a data point into the sorted array
void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count < MAX_RESISTANCE_POINTS) {
        for (int i = count; i > index; --i) {
            data[i][0] = data[i - 1][0];
            data[i][1] = data[i - 1][1];
        }
        data[index][0] = current;
        data[index][1] = resistance;
        count++;
    }
}

// Helper function to average two data points
void averageDataPoints(float data[][2], int index1, int index2) {
    data[index1][0] = (data[index1][0] + data[index2][0]) / 2.0f;
    data[index1][1] = (data[index1][1] + data[index2][1]) / 2.0f;
}

// Helper function to remove a data point at a given index
void removeDataPoint(float data[][2], int& count, int index) {
    if (index >= 0 && index < count) {
        for (int i = index; i < count - 1; ++i) {
            data[i][0] = data[i + 1][0];
            data[i][1] = data[i + 1][1];
        }
        count--;
    }
}

// Helper function to calculate the standard deviation of a vector of floats
float standardDeviation(const std::vector<float>& data) {
    if (data.empty()) return 0.0f;
    float sum = std::accumulate(data.begin(), data.end(), 0.0f);
    float mean = sum / data.size();
    std::vector<float> diffSq(data.size());
    std::transform(data.begin(), data.end(), diffSq.begin(),
                   [mean](float x){ return std::pow(x - mean, 2); });
    float sqSum = std::accumulate(diffSq.begin(), diffSq.end(), 0.0f);
    return std::sqrt(sqSum / data.size());
}

// Improved helper function to store or average resistance data with overflow handling
void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (resistance <= 0) return; // Ignore invalid or negative resistance

    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) {
            insertIndex++;
        }
        insertDataPoint(data, count, current, resistance, insertIndex);
    } else {
        int closestIndex = findClosestIndex(data, count, current);
        if (closestIndex >= 0 && closestIndex < MAX_RESISTANCE_POINTS) {

            float isolationThreshold = 0.0f; // Initialize

            if (count >= 2) {
                std::vector<float> spacings;
                for (int i = 1; i < count; ++i) {
                    spacings.push_back(data[i][0] - data[i - 1][0]);
                }

                if (!spacings.empty()) {
                    float sum = std::accumulate(spacings.begin(), spacings.end(), 0.0f);
                    float meanSpacing = sum / spacings.size();
                    float stdDevSpacing = standardDeviation(spacings);

                    // Define isolation threshold based on mean and standard deviation
                    isolationThreshold = meanSpacing + 1.5f * stdDevSpacing; // Adjust the multiplier as needed
                    if (isolationThreshold <= 0) isolationThreshold = 0.02f; // Ensure a minimum threshold
                } else {
                    isolationThreshold = 0.02f; // Default if only one spacing
                }
            } else {
                isolationThreshold = 0.02f; // Default if less than 2 points
            }

            bool isIsolated = true;
            if (count > 1) {
                float distanceToPrev = (closestIndex > 0) ? std::fabs(data[closestIndex][0] - data[closestIndex - 1][0]) : isolationThreshold * 2.0f;
                float distanceToNext = (closestIndex < count - 1) ? std::fabs(data[closestIndex][0] - data[closestIndex + 1][0]) : isolationThreshold * 2.0f;

                if (distanceToPrev < isolationThreshold || distanceToNext < isolationThreshold) {
                    isIsolated = false;
                }
            } else {
                isIsolated = false; // Not isolated if only one point exists
            }

            if (isIsolated && count >= 2) {
                int index1 = -1, index2 = -1;

                if (closestIndex > 0) {
                    index1 = closestIndex - 1;
                }
                if (closestIndex < count - 1) {
                    index2 = closestIndex + 1;
                }

                if (index1 != -1 && index2 != -1) {
                    // Average the two neighbors
                    averageDataPoints(data, index1, index2);
                    // Remove the second averaged point
                    removeDataPoint(data, count, index2);
                    // Insert the new data point at the correct sorted position
                    int insertIndex = 0;
                    while (insertIndex < count && data[insertIndex][0] < current) {
                        insertIndex++;
                    }
                    insertDataPoint(data, count, current, resistance, insertIndex);
                } else {
                    // Fallback to averaging with the closest if no two other close points are available
                    data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                    data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
                }
            } else {
                // Average the new data with the closest point if not isolated or less than 2 points exist
                data[closestIndex][1] = (data[closestIndex][1] + resistance) / 2.0f;
                data[closestIndex][0] = (data[closestIndex][0] + current) / 2.0f;
            }
        }
    }
}

// --- New function to distribute error ---
void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < 4) return; // Need at least 4 points for a cluster

    for (int i = 0; i <= count - 4; ++i) {
        // Check for a potential cluster of at least 4 points
        for (int j = i + 3; j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                // Found a cluster from index i to j (inclusive)
                std::vector<float> cluster_resistances;
                for (int k = i; k <= j; ++k) {
                    cluster_resistances.push_back(data[k][1]);
                }

                if (cluster_resistances.size() >= 4) {
                    float sum_resistance = std::accumulate(cluster_resistances.begin(), cluster_resistances.end(), 0.0f);
                    float average_resistance = sum_resistance / cluster_resistances.size();
                    float std_dev_resistance = standardDeviation(cluster_resistances);

                    std::vector<int> high_error_indices;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - average_resistance) > error_threshold_multiplier * std_dev_resistance) {
                            high_error_indices.push_back(k);
                        }
                    }

                    if (!high_error_indices.empty()) {
                        // Distribute the error by setting high error points to the average
                        for (int index : high_error_indices) {
                            data[index][1] = average_resistance;
                        }
                        // Optionally, you could implement a more sophisticated error distribution here
                        // e.g., shifting the difference proportionally to other points in the cluster.
                    }
                    // Move the outer loop index 'i' to the end of the current cluster
                    // to avoid re-processing overlapping clusters immediately.
                    i = j;
                    break; // Break the inner loop 'j' as we've processed this cluster
                }
            } else {
                break; // Points are no longer closely spaced, move to the next potential starting point 'i'
            }
        }
    }
}

// --- Linear Regression Function ---
bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) {
        Serial.println("Insufficient data points for linear regression.");
        return false;
    }

    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumXY = 0.0f;
    float sumX2 = 0.0f;

    for (int i = 0; i < count; ++i) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += data[i][0] * data[i][1];
        sumX2 += data[i][0] * data[i][0];
    }

    float n = static_cast<float>(count);
    float denominator = n * sumX2 - sumX * sumX;

    if (std::fabs(denominator) < 1e-6) { // Avoid division by zero
        Serial.println("Denominator is too small for linear regression.");
        return false;
    }

    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY - slope * sumX) / n;

    return true;
}

/**
 * Measures the MH electrode voltage by comparing unloaded vs loaded voltages
 * Returns the data structure with measurements
 */

// new asynchronous 

// Start an MH electrode measurement: non-blocking
// - testDutyCycle: duty applied for loaded sample
// - stabilization_delay: time to wait after applying load before sampling loaded voltage (ms)
// - unloaded_delay: time to wait after stopping load before sampling unloaded voltage (ms) (defaults to UNLOADED_VOLTAGE_DELAY_MS)
void startMHElectrodeMeasurement(int testDutyCycle, unsigned long stabilization_delay = STABILIZATION_DELAY_MS,
                                 unsigned long unloaded_delay = UNLOADED_VOLTAGE_DELAY_MS) {
    if (meas.active()) {
        Serial.println("Measurement already running — start ignored.");
        return;
    }
    // initialize
    meas.reset();
    meas.testDuty = testDutyCycle;
    meas.stabilizationDelay = stabilization_delay;
    meas.unloadedDelay = unloaded_delay;
    // STOP LOAD and start waiting
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    meas.stateStart = millis();
    meas.state = MEAS_STOPLOAD_WAIT;
    meas.resultReady = false;
    Serial.printf("Measurement started: testDC=%d, unst=%lums, stab=%lums\n", testDutyCycle, unmanagedCastUL(meas.unloadedDelay), unmanagedCastUL(meas.stabilizationDelay));
}

// helper for safe printing of unsigned long constants in printf
inline unsigned long unmanagedCastUL(unsigned long v){ return v; }

// Call this frequently (every 50-200 ms) — it advances the measurement state machine.
// Returns true if measurement is still active, false if idle or complete.
bool measurementStep() {
    if (meas.state == MEAS_IDLE || meas.state == MEAS_COMPLETE || meas.state == MEAS_ABORTED) return false;

    unsigned long now = millis();

    switch (meas.state) {
        case MEAS_STOPLOAD_WAIT:
        {
                dutyCycle = 0;
                analogWrite(pwmPin, dutyCycle);
          
            if (now - meas.stateStart >= meas.unloadedDelay) {
                // sample unloaded
                meas.unloadedData = MeasurementData(); // clear
                getThermistorReadings(meas.unloadedData.temp1, meas.unloadedData.temp2, meas.unloadedData.tempDiff,
                                      meas.unloadedData.t1_millivolts, meas.unloadedData.voltage, meas.unloadedData.current);
                meas.unloadedData.dutyCycle = 0;
                meas.unloadedData.timestamp = now;
                processThermistorData(meas.unloadedData, "MH idle (async)");

                // Now apply load
                dutyCycle = meas.testDuty;
                analogWrite(pwmPin, meas.testDuty);
                meas.stateStart = now;
                meas.state = MEAS_APPLY_LOAD;
            }
            break;
        };

        case MEAS_APPLY_LOAD:
        {
                //dutyCycle = meas.testDuty;
                //analogWrite(pwmPin, meas.testDuty); 
          
            if (now - meas.stateStart >= meas.stabilizationDelay) {
                // sample loaded
                meas.loadedData = MeasurementData();
                getThermistorReadings(meas.loadedData.temp1, meas.loadedData.temp2, meas.loadedData.tempDiff,
                                      meas.loadedData.t1_millivolts, meas.loadedData.voltage, meas.loadedData.current);
                meas.loadedData.dutyCycle = meas.testDuty;
                meas.loadedData.timestamp = now;
                processThermistorData(meas.loadedData, "MH loaded (async)");

                // Build result
                meas.result = MHElectrodeData();
                meas.result.unloadedVoltage = meas.unloadedData.voltage;
                meas.result.loadedVoltage = meas.loadedData.voltage;
                meas.result.current = meas.loadedData.current;
                meas.result.dutyCycle = meas.testDuty;
                meas.result.timestamp = now;
                meas.result.targetVoltage = meas.result.unloadedVoltage + (meas.result.loadedVoltage - meas.result.unloadedVoltage) * MH_ELECTRODE_RATIO;
                meas.result.voltageDifference = meas.result.loadedVoltage - meas.result.targetVoltage;

                // fallback: if current is below threshold, estimate it
                if (meas.result.current < MEASURABLE_CURRENT_THRESHOLD) {
                    meas.result.current = estimateCurrent(meas.testDuty);
                }

                meas.resultReady = true;
                meas.state = MEAS_COMPLETE;
                dutyCycle = 0;
                analogWrite(pwmPin, dutyCycle);
                Serial.printf("Measurement complete: testDC=%d U=%.3f L=%.3f I=%.3f target=%.3f diff=%.3f\n",
                              meas.testDuty, meas.result.unloadedVoltage, meas.result.loadedVoltage,
                              meas.result.current, meas.result.targetVoltage, meas.result.voltageDifference);
            }
            break;
        }

        default:
            break;
    }
    return meas.active();
}

// Query result: returns true if a result was available and copies it into out; clears the ready flag.
bool fetchMeasurementResult(MHElectrodeData &out) {
    if (!meas.resultReady) return false;
    out = meas.result;
    meas.resultReady = false;
    // NOTE: we keep dutyCycle at meas.testDuty (same behaviour as original measureMHElectrodeVoltage)
    // if you prefer to restore previous charging duty, do it here.
    return true;
}

// Abort current measurement (if needed)
void abortMeasurement() {
    if (meas.active()) {
        meas.state = MEAS_ABORTED;
        meas.resultReady = false;
        // optionally restore previous dutyCycle or safe state
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        Serial.println("Measurement aborted.");
    }
}


// call to start the whole manager (non-blocking)
void startFindOptimalManagerAsync(int maxChargeDutyCycle, int suggestedStartDutyCycle, bool isReeval) {
    findOpt = FindOptManager();
    findOpt.active = true;
    findOpt.maxDC = (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) ? MAX_CHARGE_DUTY_CYCLE : maxChargeDutyCycle;
    findOpt.lowDC = max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
    findOpt.highDC = findOpt.maxDC;
    findOpt.optimalDC = findOpt.lowDC;
    findOpt.closestVoltageDifference = 1000.0f;
    findOpt.cache.reserve(MAX_RESISTANCE_POINTS);
    findOpt.phase = FIND_INIT_HIGHDC; // start by measuring at high DC (this also gives the unloaded voltage)
    findOpt.isReevaluation = isReeval;

    // Start the measurement (this will take the unloaded sample then the highDC loaded sample)
    startMHElectrodeMeasurement(findOpt.highDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
    Serial.printf("FindOpt async started: measuring highDC=%d\n", findOpt.highDC);
}

// Ensure the phase enum includes FIND_FINAL_WAIT, e.g.:
// enum FindPhase { FIND_IDLE, FIND_INIT_HIGHDC, FIND_BINARY_PREPARE, FIND_BINARY_WAIT, FIND_FINAL_WAIT, FIND_COMPLETE };

bool findOptimalChargingDutyCycleStepAsync() {
    if (!findOpt.active) return false;

    // Advance measurement engine first
    measurementStep();

    // ---------- 1) INIT (we requested initial highDC measurement at start)
    if (findOpt.phase == FIND_INIT_HIGHDC) {
        if (meas.resultReady) {
            MHElectrodeData dataHigh;
            if (fetchMeasurementResult(dataHigh)) {
                // Save initial unloaded voltage and compute target
                findOpt.initialUnloadedVoltage = dataHigh.unloadedVoltage;
                findOpt.targetVoltage = findOpt.initialUnloadedVoltage + (dataHigh.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

                // Store initial LU internal resistance if measurable
                if (dataHigh.current > 0.01f) {
                    float internalResistanceLUInitial = (findOpt.initialUnloadedVoltage - dataHigh.loadedVoltage) / dataHigh.current;
                    storeOrAverageResistanceData(dataHigh.current, std::fabs(internalResistanceLUInitial),
                                                 internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }

                // Cache the measurement (bounded)
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(dataHigh);

                // Initialize optimal to a sane value (use low bound as starting)
                findOpt.optimalDC = max(MIN_CHARGE_DUTY_CYCLE, findOpt.lowDC);

                findOpt.phase = FIND_BINARY_PREPARE;
                Serial.printf("FindOpt init complete: Unloaded=%.3f Loaded@%d=%.3f Curr=%.3f Target=%.3f\n",
                              findOpt.initialUnloadedVoltage, dataHigh.dutyCycle, dataHigh.loadedVoltage, dataHigh.current, findOpt.targetVoltage);
            }
        }
        return true; // still active
    }

    // ---------- 2) PREPARE: decide to finish or request a mid measurement ----------
    if (findOpt.phase == FIND_BINARY_PREPARE) {
        // termination condition (same as original)
        if (findOpt.highDC - findOpt.lowDC <= CHARGE_CURRENT_STEP * 2) {
            // Before finishing, perform error distribution and regressions (same order as original)
            distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
            distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);

            if (resistanceDataCount >= 2) {
                if (performLinearRegression(internalResistanceData, resistanceDataCount,
                                            regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
            }

            if (resistanceDataCountPairs >= 2) {
                if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                            regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                    Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                  regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                }
            } else {
                Serial.println("Not enough data points for linear regression of paired resistance.");
            }

            // Schedule a final measurement at the best found duty (optimalDC)
            // If optimalDC was never updated, choose lowDC as fallback
            if (findOpt.optimalDC < MIN_CHARGE_DUTY_CYCLE) findOpt.optimalDC = findOpt.lowDC;
            int finalDC = findOpt.optimalDC;
            startMHElectrodeMeasurement(finalDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
            findOpt.phase = FIND_FINAL_WAIT;
            Serial.printf("FindOpt finishing: scheduling final measurement at %d\n", finalDC);
            return true;
        }

        // Otherwise request a mid measurement and go to WAIT
        int midDC = (findOpt.lowDC + findOpt.highDC) / 2;
        startMHElectrodeMeasurement(midDC, STABILIZATION_DELAY_MS, UNLOADED_VOLTAGE_DELAY_MS);
        findOpt.phase = FIND_BINARY_WAIT;
        Serial.printf("FindOpt: requested mid measurement DC=%d (low=%d high=%d)\n", midDC, findOpt.lowDC, findOpt.highDC);
        return true;
    }

    // ---------- 3) WAIT for a mid measurement result ----------
    if (findOpt.phase == FIND_BINARY_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData cur;
            if (fetchMeasurementResult(cur)) {
                // Fallback current estimation if under threshold
                if (cur.current < MEASURABLE_CURRENT_THRESHOLD) {
                    cur.current = estimateCurrent(cur.dutyCycle);
                }

                // Store LU internal resistance if measurable
                if (cur.current > 0.01f) {
                    float internalResistanceLU = (cur.unloadedVoltage - cur.loadedVoltage) / cur.current;
                    storeOrAverageResistanceData(cur.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }

                // Add to cache (bounded)
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(cur);

                // Pairwise internal resistance calculations against cache
                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(cur.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistancePair = (cached.loadedVoltage - cur.loadedVoltage) / (cur.current - cached.current);
                        float higherCurrent = std::max(cur.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                        
                    }
                }

                // Evaluate difference to target and update binary search bounds         
                float curDiff = fabs(cur.loadedVoltage - findOpt.targetVoltage);
                Serial.printf("FindOpt measured DC=%d loaded=%.3f target=%.3f diff=%.3f curr=%.3f\n",
                              cur.dutyCycle, cur.loadedVoltage, findOpt.targetVoltage, curDiff, cur.current);

                if (cur.loadedVoltage < findOpt.targetVoltage) {
                    // loaded < target -> increase duty (move low up)
                    findOpt.lowDC = cur.dutyCycle;
                } else {
                    // loaded >= target -> decrease duty (move high down)
                    findOpt.highDC = cur.dutyCycle;
                }

                if (curDiff < findOpt.closestVoltageDifference) {
                    findOpt.closestVoltageDifference = curDiff;
                    findOpt.optimalDC = cur.dutyCycle;
                }

                // store previous and continue
                findOpt.phase = FIND_BINARY_PREPARE;

            }
        }
        return true;
    }

    // ---------- 4) WAIT for the final measurement, then finalize ----------
    if (findOpt.phase == FIND_FINAL_WAIT) {
        if (meas.resultReady) {
            MHElectrodeData finalData;
            if (fetchMeasurementResult(finalData)) {
                // Store any final LU resistance from final measurement
                if (finalData.current < MEASURABLE_CURRENT_THRESHOLD) {
                    finalData.current = estimateCurrent(finalData.dutyCycle);
                }
                if (finalData.current > 0.01f) {
                    float internalResistanceLU = (finalData.unloadedVoltage - finalData.loadedVoltage) / finalData.current;
                    storeOrAverageResistanceData(finalData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                }

                // Cache final point
                if ((int)findOpt.cache.size() >= MAX_RESISTANCE_POINTS) findOpt.cache.erase(findOpt.cache.begin());
                findOpt.cache.push_back(finalData);

                // Pairwise resistance calc for final point
                for (const auto& cached : findOpt.cache) {
                    if (std::fabs(finalData.current - cached.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                        float internalResistancePair = (cached.loadedVoltage - finalData.loadedVoltage) / (finalData.current - cached.current);
                        float higherCurrent = std::max(finalData.current, cached.current);
                        storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair),
                                                     internalResistanceDataPairs, resistanceDataCountPairs);
                        bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    }
                }

                // Re-run final error distribution & regressions to ensure final model includes last point:
                distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f);
                distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f);

                if (resistanceDataCount >= 2) {
                    if (performLinearRegression(internalResistanceData, resistanceDataCount,
                                                regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
                        Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                                      regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
                    }
                } else {
                    Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
                }

                if (resistanceDataCountPairs >= 2) {
                    if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs,
                                                regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
                        Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                                      regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
                    }
                } else {
                    Serial.println("Not enough data points for linear regression of paired resistance.");
                }

                // Compute the final targetVoltage (mirror original)
 //               findOpt.targetVoltage = findOpt.initialUnloadedVoltage + (finalData.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

               float Rint_est = (finalData.current > 0.01f)
                                 ? fabs((finalData.unloadedVoltage - finalData.loadedVoltage) / finalData.current)
                                 : 0.0f;

                // Normal target computation
                float unclampedTarget = findOpt.initialUnloadedVoltage +
                                        (finalData.loadedVoltage - findOpt.initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

                // Clamp again to maximumCurrent
                if (Rint_est > 0 && maximumCurrent > 0) {
                    float maxDrop = Rint_est * maximumCurrent;
                    float maxAllowedVoltage = findOpt.initialUnloadedVoltage + maxDrop;
                    findOpt.targetVoltage = constrain(unclampedTarget, findOpt.initialUnloadedVoltage, maxAllowedVoltage);
                } else {
                    findOpt.targetVoltage = unclampedTarget;
                }


                Serial.printf("FindOpt complete: optimalDC=%d loaded=%.3f target=%.3f diff=%.3f\n",
                              findOpt.optimalDC, finalData.loadedVoltage, findOpt.targetVoltage,
                              fabs(finalData.loadedVoltage - findOpt.targetVoltage));

                // Publish final result to accessible globals
                cachedOptimalDuty = findOpt.optimalDC;
                findOpt.active = false;
                findOpt.phase = FIND_COMPLETE;

                // ensure caller sees completion by returning false below
                return false;
            }
        }
        return true; // still waiting for final measurement to complete
    }

    // If code reaches here, keep active by default
    return true;
}



//old synchronous
 
MHElectrodeData measureMHElectrodeVoltage(int testDutyCycle) {
    MHElectrodeData data;

    // Measure unloaded voltage
    MeasurementData unloadedData = getUnloadedVoltageMeasurement();
    processThermistorData(unloadedData, "MH idle");
    data.unloadedVoltage = unloadedData.voltage;

    // Measure loaded voltage with the specified duty cycle
    MeasurementData loadedData = takeMeasurement(testDutyCycle, STABILIZATION_DELAY_MS);
    processThermistorData(loadedData, "MH loaded");
    data.loadedVoltage = loadedData.voltage;
    data.current = loadedData.current;
    data.dutyCycle = testDutyCycle;

    // Calculate the target voltage
    data.targetVoltage = data.unloadedVoltage + (data.loadedVoltage - data.unloadedVoltage) * MH_ELECTRODE_RATIO;

    // Calculate the difference between loaded and target voltage
    data.voltageDifference = data.loadedVoltage - data.targetVoltage;

    return data;
}

/**
 * Finds the optimal charging duty cycle that maintains the MH electrode voltage
 * at the target voltage (half way between unloaded and loaded).
 */
// --- Modified findOptimalChargingDutyCycle function with caching ---
int findOptimalChargingDutyCycle(int maxChargeDutyCycle, int suggestedStartDutyCycle) {

//    Serial.println("building duty cycle to current model...");
//    bigUglyMessage("duty cycle model");
//    buildCurrentModel(true); // Accumulate more data and refine the model

    Serial.println("Finding optimal charging duty cycle...");
    bigUglyMessage("finding duty cycle");

    if (maxChargeDutyCycle < MIN_CHARGE_DUTY_CYCLE) {
        Serial.println("Error: maxChargeDutyCycle is invalid, using MAX_CHARGE_DUTY_CYCLE instead.");
        maxChargeDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    }

    int optimalDutyCycle = MIN_CHARGE_DUTY_CYCLE;
    float closestVoltageDifference = 1000.0f; // Initialize with a large value
    float targetVoltage = 0.0f; // Will be updated in the loop

    // Start with a binary search approach to narrow down the range
    int lowDC = MIN_CHARGE_DUTY_CYCLE;
    int highDC = maxChargeDutyCycle;

    // First, measure the unloaded voltage once
    MeasurementData initialUnloadedData = getUnloadedVoltageMeasurement();
    processThermistorData(initialUnloadedData, "MH idle (initial)");
    float initialUnloadedVoltage = initialUnloadedData.voltage;

    // Measure loaded voltage at maximum duty cycle
    MHElectrodeData dataHigh = measureMHElectrodeVoltage(highDC);

    // Calculate and store internal resistance during initial measurement at max duty cycle (Loaded/Unloaded)
    if (dataHigh.current > 0.01f) {
        float internalResistanceLUInitial = (initialUnloadedVoltage - dataHigh.loadedVoltage) / dataHigh.current;
        storeOrAverageResistanceData(dataHigh.current, std::fabs(internalResistanceLUInitial), internalResistanceData, resistanceDataCount);
        bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
    }

    // Recalculate target voltage based on the initial unloaded voltage and max duty cycle possible
    targetVoltage = initialUnloadedVoltage + (dataHigh.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    MHElectrodeData previousData = dataHigh; // Store the result of the max duty cycle measurement
    bool firstIteration = true;

    Serial.printf("Initial Measurement (Max DC %d): Unloaded: %.3fV, Loaded: %.3fV, Current: %.3fA\n",
                  highDC, initialUnloadedVoltage, dataHigh.loadedVoltage, dataHigh.current);

    // --- Introduce a cache for MHElectrodeData ---
    std::vector<MHElectrodeData> dataCache;
    const int MAX_CACHE_SIZE = MAX_RESISTANCE_POINTS; // Adjust the cache size as needed

    auto addToCache = [&](const MHElectrodeData& data) {
        if (dataCache.size() >= MAX_CACHE_SIZE) {
            dataCache.erase(dataCache.begin()); // Remove the oldest element
        }
        dataCache.push_back(data);
    };

    // Add the initial high duty cycle measurement to the cache
    addToCache(dataHigh);

    // Set the starting point for the binary search
    lowDC = max(MIN_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);

    if (lowDC > highDC) {
        lowDC = MIN_CHARGE_DUTY_CYCLE; // Ensure lowDC is not greater than highDC
    }

    while (highDC - lowDC > CHARGE_CURRENT_STEP * 2) {
        int midDC = (lowDC + highDC) / 2;

        MHElectrodeData currentData = measureMHElectrodeVoltage(midDC);
        if (currentData.current < MEASURABLE_CURRENT_THRESHOLD) {currentData.current=static_cast<double>(estimateCurrent(midDC));}; // replace with inference if below threshold

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
        }

        // --- Check cache for more opportunities to calculate internal resistance (Pair) ---
        for (const auto& cachedData : dataCache) {
            if (std::fabs(currentData.current - cachedData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                float internalResistancePair = (cachedData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - cachedData.current);
                float higherCurrent = std::max(currentData.current, cachedData.current);
                storeOrAverageResistanceData(higherCurrent, std::fabs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
                bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs); // Sort after adding
            }
        }

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Binary Search - Duty Cycle: %d, Unloaded: %.3fV, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      midDC, currentData.unloadedVoltage, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentData.loadedVoltage < targetVoltage) {
            // Loaded voltage is below target, increase duty cycle to increase load (and voltage)
            lowDC = midDC;
        } else {
            // Loaded voltage is above target, decrease duty cycle to decrease load (and voltage)
            highDC = midDC;
        }

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = midDC;
        }

        previousData = currentData;
        firstIteration = false;
    }

/*
    // Fine tune with small steps around the approximate value
    for (int dc = max(MIN_CHARGE_DUTY_CYCLE, optimalDutyCycle - CHARGE_CURRENT_STEP * 3);
         dc <= min(MAX_CHARGE_DUTY_CYCLE, optimalDutyCycle + CHARGE_CURRENT_STEP * 3);
         dc += CHARGE_CURRENT_STEP) {

        MHElectrodeData currentData = measureMHElectrodeVoltage(dc);

        // Add current data to the cache
        addToCache(currentData);

        // Calculate and store internal resistance (Loaded/Unloaded)
        if (currentData.current > 0.01f) {
            float internalResistanceLU = (currentData.unloadedVoltage - currentData.loadedVoltage) / currentData.current;
            storeOrAverageResistanceData(currentData.current, std::fabs(internalResistanceLU), internalResistanceData, resistanceDataCount);
            bubbleSort(internalResistanceData, resistanceDataCount); // Sort after adding
        }

        // --- Check cache for more opportunities to calculate internal resistance (Pair) ---
        for (const auto& cachedData : dataCache) {
            if (std::abs(currentData.current - cachedData.current) > MIN_CURRENT_DIFFERENCE_FOR_PAIR) {
                float internalResistancePair = (cachedData.loadedVoltage - currentData.loadedVoltage) / (currentData.current - cachedData.current);
                float higherCurrent = std::max(currentData.current, cachedData.current);
                storeOrAverageResistanceData(higherCurrent, std::abs(internalResistancePair), internalResistanceDataPairs, resistanceDataCountPairs);
                bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs); // Sort after adding
            }
        }

        float currentVoltageDifference = fabs(currentData.loadedVoltage - targetVoltage);

        Serial.printf("Fine-tuning - Duty Cycle: %d, Loaded: %.3fV, Target: %.3fV, Diff: %.3fV, Current: %.3fA\n",
                      dc, currentData.loadedVoltage, targetVoltage, currentVoltageDifference, currentData.current);

        if (currentVoltageDifference < closestVoltageDifference) {
            closestVoltageDifference = currentVoltageDifference;
            optimalDutyCycle = dc;
        }
        previousData = currentData;
    }
*/

    // Perform error distribution on the collected Loaded/Unloaded resistance data
    distribute_error(internalResistanceData, resistanceDataCount, 0.05f, 1.05f); // Example parameters, adjust as needed

    // Perform error distribution on the collected paired resistance data
    distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, 0.05f, 1.05f); // Example parameters, adjust as needed

    // Final measurement at the optimal duty cycle
    MHElectrodeData finalData = measureMHElectrodeVoltage(optimalDutyCycle);
    targetVoltage = initialUnloadedVoltage + (finalData.loadedVoltage - initialUnloadedVoltage) * MH_ELECTRODE_RATIO;

    Serial.printf("Optimal charging duty cycle found: %d (loaded: %.3fV, target: %.3fV, diff: %.3fV)\n",
                  optimalDutyCycle, finalData.loadedVoltage, targetVoltage, fabs(finalData.loadedVoltage - targetVoltage));

    // Perform linear regression after finding the optimal duty cycle
    if (resistanceDataCount >= 2) {
        if (performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept)) {
            Serial.printf("Regressed Internal Resistance (Loaded/Unloaded): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of Loaded/Unloaded resistance.");
    }

    if (resistanceDataCountPairs >= 2) {
        if (performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept)) {
            Serial.printf("Regressed Internal Resistance (Pairs): Slope = %.4f, Intercept = %.4f\n",
                          regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
        }
    } else {
        Serial.println("Not enough data points for linear regression of paired resistance.");
    }

    return optimalDutyCycle;
}

// end of old

// new






// --- Helpers ---
static float computeDissipatedPower(float vUnderLoad, float vNoLoad, float current, float Rparam) {
  const float epsI = 1e-9f;
  float P_v = 0.0f;
  if (current > epsI) {
    float vDrop = vNoLoad - vUnderLoad; // ideally Voc - Vterm
    if (vDrop > 1e-6f) {
      // P dissipated internally = I * voltage drop across internal elements
      P_v = current * vDrop;
    }
  }
  float P_r = 0.0f;
  if (Rparam > 0.0f) {
    P_r = current * current * Rparam;
  }
  // If both available, average them (reduces sensitivity to measurement noise).
  if (P_v > 0.0f && P_r > 0.0f) return 0.5f * (P_v + P_r);
  return (P_v > 0.0f) ? P_v : P_r; // one of them (or zero)
}

static float thermalConductance_W_per_K(float area, float h, float emissivity, float T_ambientK) {
  // linearized radiative term: ~4 * eps * sigma * A * T^3
  return h * area + 4.0f * emissivity * STEFAN_BOLTZMANN * area * powf(T_ambientK, 3.0f);
}

// --- Main: update battery temperature given previous battery temperature ---
// Returns new battery temperature in °C.
float estimateTempDiff(
  float voltageUnderLoad,
  float voltageNoLoad,
  float current,                 // A
  float internalResistanceParam, // Ω: pass 0 if you want it estimated from voltages
  float ambientTempC,            // °C
  uint32_t currentTime,          // millis()
  uint32_t lastChargeEvaluationTime, // millis()
  float BatteryTempC,    // °C (state you maintain across calls)
  // optional thermal parameters with defaults
  float cellMassKg = DEFAULT_CELL_MASS_KG,
  float specificHeat = DEFAULT_SPECIFIC_HEAT,
  float area = DEFAULT_SURFACE_AREA_M2,
  float convectiveH = DEFAULT_CONVECTIVE_H,
  float emissivity = DEFAULT_EMISSIVITY
) {
  // compute dt in seconds (works correctly across millis() rollover thanks to unsigned arithmetic)
  uint32_t dt_ms = (uint32_t)(currentTime - lastChargeEvaluationTime);
  float dt_s = (float)dt_ms * 0.001f;
  if (dt_s <= 0.0f) return BatteryTempC; // no time elapsed

  // Estimate dissipated power (W)
  float P = computeDissipatedPower(voltageUnderLoad, voltageNoLoad, current, internalResistanceParam);

  // Convert ambient to Kelvin
  float T_amb_K = ambientTempC + 273.15f;

  // Thermal conductance (W/K)
  float G = thermalConductance_W_per_K(area, convectiveH, emissivity, T_amb_K);

  // Thermal capacitance (J/K)
  float Cth = cellMassKg * specificHeat; // (m * c)

  // If G is extremely small (isolated), fall back to pure energy accumulation: dT = P * dt / Cth
  const float tiny = 1e-12f;
  if (G <= tiny) {
    float deltaT = (P * dt_s) / Cth;
    return BatteryTempC + deltaT;
  }

  // steady-state theta (K) for given power: theta_ss = P / G
  float theta_ss = P / G;

  // initial theta (relative to ambient)
  float theta0 = BatteryTempC - ambientTempC;

  // thermal time constant tau = Cth / G (seconds)
  float tau = Cth / G;

  // analytical solution for linear ODE:
  // theta(t+dt) = theta_ss + (theta0 - theta_ss) * exp(-dt / tau)
  float expo = expf(-dt_s / tau);
  float theta_new = theta_ss + (theta0 - theta_ss) * expo;

  //float newTempC = ambientTempC + theta_new;
  //  return newTempC;  //do not return absolute temperature
  return theta_new; // return only difference instead.
}

//------------------------auto label placing log graph

// ---------- Improved / corrected plotting helpers + drawChargePlot(bool,bool) ----------
// Replace your existing helper functions and drawChargePlot with this block.
// Required elsewhere in your project:
//  - tft object (TFT_eSPI or equivalent) with used methods
//  - std::vector<ChargeLogData> chargeLog;
//  - estimateTempDiff(...)
//  - regressedInternalResistancePairsIntercept
//  - MAX_TEMP_DIFF_THRESHOLD
//  - darkerColor(uint16_t, float)
//  - TFT color constants (TFT_BLACK, TFT_WHITE, TFT_DARKGREY, etc.)
//  - Label struct and LabelVerticalPlacement enum/type as in your original code

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstdlib>
#include <limits>
//#include <Eigen/Dense> // keep if your project already uses Eigen; otherwise replace with std::array

// keep the exact GRID_SIZE you used
constexpr int GRID_SIZE = 18;
using ScoreType = int8_t;

// pixel -> grid index (returns {-1,-1} if outside)
std::pair<int,int> pixelToGrid(int x, int y, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    if (x < marginX || x >= marginX + plotAreaWidth || y < marginYTop || y >= marginYTop + plotAreaHeight) {
        return {-1, -1};
    }
    int row = static_cast<int>((static_cast<float>(y - marginYTop) / static_cast<float>(plotAreaHeight)) * GRID_SIZE);
    int col = static_cast<int>((static_cast<float>(x - marginX) / static_cast<float>(plotAreaWidth)) * GRID_SIZE);
    row = std::min(GRID_SIZE - 1, std::max(0, row));
    col = std::min(GRID_SIZE - 1, std::max(0, col));
    return {row, col};
}

// grid index -> pixel center
std::pair<int,int> gridToPixel(int row, int col, int plotAreaWidth, int plotAreaHeight, int marginX, int marginYTop) {
    float cellHeight = static_cast<float>(plotAreaHeight) / GRID_SIZE;
    float cellWidth  = static_cast<float>(plotAreaWidth)  / GRID_SIZE;
    int y = static_cast<int>(marginYTop + (row + 0.5f) * cellHeight);
    int x = static_cast<int>(marginX    + (col + 0.5f) * cellWidth);
    return {x, y};
}

int calculateVerticalPosition(int yInitial, int textHeight, LabelVerticalPlacement placement) {
    switch (placement) {
        case LabelVerticalPlacement::CENTER:
            return yInitial - textHeight / 2;
        case LabelVerticalPlacement::ABOVE:
            return yInitial - textHeight - 2;
        case LabelVerticalPlacement::BELOW:
            return yInitial + 2;
        default:
            return yInitial - textHeight / 2;
    }
}

// The single, backward-compatible API
void drawChargePlot(bool autoscaleX, bool autoscaleY) {
    // Ensure the global chargeLog exists in your sketch:
    extern std::vector<ChargeLogData> chargeLog;

    if (chargeLog.empty()) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(6); // MC_DATUM
        tft.drawString("No charge data to plot", tft.width() / 2, tft.height() / 2, 2);
        return;
    }

    tft.fillScreen(TFT_BLACK);

    int plotAreaWidth  = tft.width() - 40;
    int plotAreaHeight = tft.height() - 60; // leave room for legend
    int marginX        = 20;
    int marginYTop     = 20;

    #ifdef DEBUG_LABELS
    Serial.println("--- Starting drawChargePlot ---");
    Serial.print("Plot Area Width: "); Serial.println(plotAreaWidth);
    Serial.print("Plot Area Height: "); Serial.println(plotAreaHeight);
    Serial.print("Margin X: "); Serial.println(marginX);
    Serial.print("Margin Y Top: "); Serial.println(marginYTop);
    #endif

    // --- X scaling (timestamps -> x pixels) ---
    unsigned long startTime = chargeLog.front().timestamp;
    unsigned long endTime   = chargeLog.back().timestamp;
    double timeScaleX = 1.0;
    if (autoscaleX && endTime > startTime) {
        // Use double to avoid integer overflow when multiplying timestamps.
        timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
    } else if (!autoscaleX) {
        unsigned long window = 10UL * 60UL * 1000UL; // 10 minutes in ms
        // keep startTime at latest window
        long long maybeStart = static_cast<long long>(endTime) - static_cast<long long>(window);
        if (maybeStart > static_cast<long long>(startTime)) startTime = static_cast<unsigned long>(maybeStart);
        if (endTime > startTime) {
            timeScaleX = static_cast<double>(plotAreaWidth) / static_cast<double>(endTime - startTime);
        } else {
            timeScaleX = 1.0;
        }
    }

    // --- Y scaling: compute min/max for each plotted series ---
    float currentMin = 1000.0f, currentMax = -1000.0f;
    float voltageMin = 1000.0f, voltageMax = -1000.0f;
    float dutyCycleMin = 1000.0f, dutyCycleMax = -1000.0f;
    float tempDiffMin = 1000.0f, tempDiffMax = -1000.0f;
    float estTempDiffThresholdMin = 1000.0f, estTempDiffThresholdMax = -1000.0f;
    float irLU_Min = 1000.0f, irLU_Max = -1000.0f;
    float irPairs_Min = 1000.0f, irPairs_Max = -1000.0f;

    if (autoscaleY) {
        for (const auto &logEntry : chargeLog) {
            currentMin = std::fmin(currentMin, logEntry.current);
            currentMax = std::fmax(currentMax, logEntry.current);

            voltageMin = std::fmin(voltageMin, logEntry.voltage);
            voltageMax = std::fmax(voltageMax, logEntry.voltage);

            dutyCycleMin = std::fmin(dutyCycleMin, static_cast<float>(logEntry.dutyCycle));
            dutyCycleMax = std::fmax(dutyCycleMax, static_cast<float>(logEntry.dutyCycle));

            float currentTempDiff = logEntry.batteryTemperature - logEntry.ambientTemperature;
            tempDiffMin = std::fmin(tempDiffMin, currentTempDiff);
            tempDiffMax = std::fmax(tempDiffMax, currentTempDiff);

            // original code used an estimatedDiff placeholder; keep behavior but avoid index bugs
            float estimatedDiff = currentTempDiff + 3.0f; // temporary fallback
            float thresholdValue = MAX_TEMP_DIFF_THRESHOLD + estimatedDiff;
            estTempDiffThresholdMin = std::fmin(estTempDiffThresholdMin, currentTempDiff);
            estTempDiffThresholdMax = std::fmax(estTempDiffThresholdMax, thresholdValue * 0.5f);

            irLU_Min   = std::fmin(irLU_Min,   logEntry.internalResistancePairs);
            irLU_Max   = std::fmax(irLU_Max,   logEntry.internalResistanceLoadedUnloaded);

            irPairs_Min = std::fmin(irPairs_Min, logEntry.internalResistancePairs);
            irPairs_Max = std::fmax(irPairs_Max, logEntry.internalResistanceLoadedUnloaded);
        }

        // avoid exact equals -> pad ranges
        auto padIfEqual = [](float &a, float &b, float pad) {
            if (a >= b) { a -= pad; b += pad; }
        };
        padIfEqual(currentMin, currentMax, 0.1f);
        padIfEqual(voltageMin, voltageMax, 0.1f);
        padIfEqual(dutyCycleMin, dutyCycleMax, 1.0f);
        padIfEqual(tempDiffMin, tempDiffMax, 0.1f);
        padIfEqual(estTempDiffThresholdMin, estTempDiffThresholdMax, 0.1f);
        padIfEqual(irLU_Min, irLU_Max, 0.01f);
        padIfEqual(irPairs_Min, irPairs_Max, 0.01f);
    } else {
        currentMin = 0.0f; currentMax = 0.4f;
        voltageMin = 1.0f; voltageMax = 2.0f;
        dutyCycleMin = 0.0f; dutyCycleMax = 255.0f;
        tempDiffMin = -0.5f; tempDiffMax = 1.5f;
        estTempDiffThresholdMin = -0.5f; estTempDiffThresholdMax = 1.5f;
        irLU_Min = 0.0f; irLU_Max = 1.5f;
        irPairs_Min = 0.0f; irPairs_Max = 1.5f;
    }

    // scaling helpers
    float scaleY = static_cast<float>(plotAreaHeight);
    auto scaleValue = [&](float val, float minVal, float maxVal) -> int {
        if (maxVal <= minVal) return marginYTop + static_cast<int>(scaleY/2.0f);
        float normalized = (val - minVal) / (maxVal - minVal); // 0..1
        float pixelFromTop = (1.0f - normalized) * scaleY;
        return marginYTop + static_cast<int>(std::round(pixelFromTop));
    };
    auto inverseScaleValue = [&](int yPixel, float minVal, float maxVal) -> float {
        if (maxVal <= minVal) return (minVal + maxVal) * 0.5f;
        float normalizedY = static_cast<float>(yPixel - marginYTop) / scaleY; // 0..1
        return minVal + (1.0f - normalizedY) * (maxVal - minVal);
    };

    // Grid scores
    Eigen::Matrix<ScoreType, GRID_SIZE, GRID_SIZE> grid;
    grid.setZero();

    // Draw axes (original: X axis at top)
    tft.drawLine(marginX, marginYTop, marginX + plotAreaWidth, marginYTop, TFT_DARKGREY); // X axis (top)
    tft.drawLine(marginX, marginYTop, marginX, marginYTop + plotAreaHeight, TFT_DARKGREY); // Y axis (left)

    // Helper: convert timestamp -> X pixel safely
    auto timeToX = [&](unsigned long t) -> int {
        if (endTime <= startTime) return marginX;
        double dt = static_cast<double>(static_cast<long long>(t) - static_cast<long long>(startTime));
        double xp = static_cast<double>(marginX) + dt * timeScaleX;
        return static_cast<int>(std::round(xp));
    };

    // Bresenham + grid update (robust and inclusive of endpoint)
    auto drawLineAndUpdateGrid = [&](int x0, int y0, int x1, int y1, uint16_t color) {
        // Draw on TFT
        tft.drawLine(x0, y0, x1, y1, color);

        // Bresenham integer algorithm
        int dx = std::abs(x1 - x0);
        int sx = (x0 < x1) ? 1 : -1;
        int dy = -std::abs(y1 - y0);
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy; // error term

        int x = x0;
        int y = y0;

        while (true) {
            auto g = pixelToGrid(x, y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
            if (g.first != -1) {
                // increment with clamp to avoid overflow of signed int8
                if (grid(g.first, g.second) < std::numeric_limits<ScoreType>::max()) {
                    grid(g.first, g.second) = static_cast<ScoreType>(grid(g.first, g.second) + 1);
                }
            }
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    };

    // Plot each series and update grid
    for (size_t i = 2; i < chargeLog.size(); ++i) { // start at 2 as your code expects history
        int x1 = timeToX(chargeLog[i - 1].timestamp);
        int x2 = timeToX(chargeLog[i].timestamp);

        // Current
        {
            int y1 = scaleValue(chargeLog[i - 1].current, currentMin, currentMax);
            int y2 = scaleValue(chargeLog[i].current, currentMin, currentMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_MAGENTA);
        }

        // Voltage
        {
            int y1 = scaleValue(chargeLog[i - 1].voltage, voltageMin, voltageMax);
            int y2 = scaleValue(chargeLog[i].voltage, voltageMin, voltageMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_YELLOW);
        }

        // Duty Cycle
        {
            int y1 = scaleValue(static_cast<float>(chargeLog[i - 1].dutyCycle), dutyCycleMin, dutyCycleMax);
            int y2 = scaleValue(static_cast<float>(chargeLog[i].dutyCycle), dutyCycleMin, dutyCycleMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_DARKGREY);
        }

        // Temperature difference
        {
            float currentTempDiffPrev = chargeLog[i - 1].batteryTemperature - chargeLog[i - 1].ambientTemperature;
            float currentTempDiffCurr = chargeLog[i].batteryTemperature     - chargeLog[i].ambientTemperature;
            int y1 = scaleValue(currentTempDiffPrev, tempDiffMin, tempDiffMax);
            int y2 = scaleValue(currentTempDiffCurr, tempDiffMin, tempDiffMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_BLUE);
        }

        // Estimated Temperature Difference Threshold
        {
            size_t prevIdx = i - 1;
            size_t prevPrevIdx = (i >= 2) ? (i - 2) : prevIdx; // safe fallback
            // call estimateTempDiff with safe indices (match your function signature)
            float estimatedDiffPrev = estimateTempDiff(
                chargeLog[prevIdx].voltage,
                chargeLog[prevIdx].voltage,
                chargeLog[prevIdx].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[prevIdx].ambientTemperature,
                chargeLog[prevIdx].timestamp,
                chargeLog[prevPrevIdx].timestamp,
                chargeLog[prevIdx].batteryTemperature
            );

            float estimatedDiffCurr = estimateTempDiff(
                chargeLog[i].voltage,
                chargeLog[i].voltage,
                chargeLog[i].current,
                regressedInternalResistancePairsIntercept,
                chargeLog[i].ambientTemperature,
                chargeLog[i].timestamp,
                chargeLog[i - 1].timestamp,
                chargeLog[i].batteryTemperature
            );

            float thresholdValuePrev = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffPrev;
            float thresholdValueCurr = MAX_TEMP_DIFF_THRESHOLD + estimatedDiffCurr;
            int y1 = scaleValue(thresholdValuePrev, estTempDiffThresholdMin, estTempDiffThresholdMax);
            int y2 = scaleValue(thresholdValueCurr, estTempDiffThresholdMin, estTempDiffThresholdMax);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_RED);
        }

        // Internal Resistance Loaded/Unloaded
        {
            int y1 = scaleValue(chargeLog[i - 1].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            int y2 = scaleValue(chargeLog[i].internalResistanceLoadedUnloaded, irLU_Min, irLU_Max);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_ORANGE);
        }

        // Internal Resistance Pairs
        {
            int y1 = scaleValue(chargeLog[i - 1].internalResistancePairs, irPairs_Min, irPairs_Max);
            int y2 = scaleValue(chargeLog[i].internalResistancePairs, irPairs_Min, irPairs_Max);
            drawLineAndUpdateGrid(x1, y1, x2, y2, TFT_CYAN);
        }
    } // end plotting loop

    #ifdef DEBUG_LABELS
    Serial.println("--- Grid Scores after Plotting ---");
    for (int r = 0; r < GRID_SIZE; ++r) {
        Serial.print("Row "); Serial.print(r); Serial.print(": ");
        for (int c = 0; c < GRID_SIZE; ++c) {
            Serial.print(static_cast<int>(grid(r, c)));
            Serial.print(" ");
        }
        Serial.println();
    }
    #endif

    // --- Label creation & placement (kept your original approach, with safe index fixes) ---
    std::vector<Label> labels;
    float labelLineLengthFactor = 0.10f;
    const int LABEL_LINE_LENGTH = 30;
    int maxLabelLineLength = static_cast<int>(plotAreaWidth * 0.1f);
    float darkeningFactor = 0.0f;
    int textSpacing = 0;
    int tickLength = 10;
    int supplementaryTickLength = 5;
    int labelPadding = 3;
    int labelMarkingScore = 5;
    int labelDisplayThreshold = 4;

    std::map<std::string, std::pair<float, float>> hardcodedRanges = {
        {"Current", {0.0f, 0.4f}},
        {"Voltage", {0.0f, 2.5f}},
        {"Duty", {0.0f, 255.0f}},
        {"TempDiff", {-0.5f, 2.0f}},
        {"TempDiffThresh", {-0.5f, 2.5f}},
        {"IR_LU", {0.0f, 2.0f}},
        {"IR_Pairs", {0.0f, 2.0f}}
    };

    auto calculateDynamicLineLength = [&](float value, float minHardcoded, float maxHardcoded) {
        if (maxHardcoded <= minHardcoded) return tickLength;
        float normalizedValue = std::abs(value - minHardcoded) / (maxHardcoded - minHardcoded);
        return static_cast<int>(normalizedValue * maxLabelLineLength);
    };

    // createLabel: uses your tft font metrics (textWidth/fontHeight)
    auto createLabel = [&](const std::string& name, float value, float minValue, float maxValue, uint16_t color) {
        float scaledYValue = scaleValue(value, minValue, maxValue);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        std::string text = ss.str();

        uint16_t darkColor = color;
        if (darkeningFactor > 0) { darkColor = darkerColor(color, darkeningFactor); }

        int tw = tft.textWidth(text.c_str(), 1);
        int th = tft.fontHeight(1);

        Label label;
        label.text = text;

        float dataRange = maxValue - minValue;
        float normalizedY = (value - minValue) / dataRange;
        int plotYPixel = marginYTop + static_cast<int>((1.0f - normalizedY) * plotAreaHeight);

        label.y = plotYPixel;
        label.color = darkColor;
        label.textWidth = tw;
        label.textHeight = th;
        label.lineY = plotYPixel;
        label.lineStartX = 0;
        label.lineEndX = 0;
        label.y_initial = plotYPixel;
        label.minValue = minValue;
        label.maxValue = maxValue;

        if (plotYPixel < marginYTop + 30) {
            label.verticalPlacement = LabelVerticalPlacement::BELOW;
        } else if (plotYPixel > marginYTop + plotAreaHeight - 30) {
            label.verticalPlacement = LabelVerticalPlacement::ABOVE;
        } else {
            label.verticalPlacement = LabelVerticalPlacement::CENTER;
        }
        return label;
    };

    // Add min/max labels (same set as original)
    labels.push_back(createLabel("Current", currentMax, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Current", currentMin, currentMin, currentMax, TFT_MAGENTA));
    labels.push_back(createLabel("Voltage", voltageMax, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("Voltage", voltageMin, voltageMin, voltageMax, TFT_YELLOW));
    labels.push_back(createLabel("DutyCycle", dutyCycleMax, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("DutyCycle", dutyCycleMin, dutyCycleMin, dutyCycleMax, TFT_DARKGREY));
    labels.push_back(createLabel("TempDiff", tempDiffMax, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("TempDiff", tempDiffMin, tempDiffMin, tempDiffMax, TFT_BLUE));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMax, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED));
    labels.push_back(createLabel("EstTempDiffThreshold", estTempDiffThresholdMin, estTempDiffThresholdMin, estTempDiffThresholdMax, TFT_RED));
    labels.push_back(createLabel("IrLU", irLU_Max, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrLU", irLU_Min, irLU_Min, irLU_Max, TFT_ORANGE));
    labels.push_back(createLabel("IrPairs", irPairs_Max, irPairs_Min, irPairs_Max, TFT_CYAN));
    labels.push_back(createLabel("IrPairs", irPairs_Min, irPairs_Min, irPairs_Max, TFT_CYAN));

    // label placement helpers (unchanged but safe)
    auto canPlaceLabel = [&](const Label& label, int gridRow, int gridCol) {
        std::pair<int, int> gridPixelPos = gridToPixel(gridRow, gridCol, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        int gridPixelXCenter = gridPixelPos.first;
        int gridPixelYCenter = gridPixelPos.second;
        int textX;
        int textY;

        if (gridCol < GRID_SIZE / 2) {
            textX = gridPixelXCenter + LABEL_LINE_LENGTH + 2;
        } else {
            textX = gridPixelXCenter - label.textWidth - LABEL_LINE_LENGTH - 2;
        }

        if (gridRow < GRID_SIZE / 2) {
            textY = gridPixelYCenter;
        } else {
            textY = gridPixelYCenter - label.textHeight;
        }

        int labelTop = textY - labelPadding;
        int labelBottom = textY + label.textHeight + labelPadding;
        int labelLeft = textX - labelPadding;
        int labelRight = textX + label.textWidth + labelPadding;

        int startRow = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int endRow   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int startCol = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
        int endCol   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

        for (int r = startRow; r <= endRow; ++r)
            for (int c = startCol; c <= endCol; ++c)
                if (grid(r, c) > labelDisplayThreshold) return false;

        return true;
    };

    auto markLabelAsPlaced = [&](const Label& label, int gridRow, int gridCol) {
        int labelTop = label.y;
        int labelBottom = label.y + label.textHeight;
        int labelLeft = label.x;
        int labelRight = label.x + label.textWidth;

        int startRow = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelTop - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int endRow   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelBottom - marginYTop) / plotAreaHeight) * GRID_SIZE)));
        int startCol = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelLeft - marginX) / plotAreaWidth) * GRID_SIZE)));
        int endCol   = std::fmax(0, std::fmin(GRID_SIZE - 1, static_cast<int>((static_cast<float>(labelRight - marginX) / plotAreaWidth) * GRID_SIZE)));

        for (int r = startRow; r <= endRow; ++r) {
            for (int c = startCol; c <= endCol; ++c) {
                if (r >= 0 && r < GRID_SIZE && c >= 0 && c < GRID_SIZE) {
                    int sum = static_cast<int>(grid(r, c)) + labelMarkingScore;
                    grid(r, c) = static_cast<ScoreType>(std::min(127, sum));
                }
            }
        }
    };

    auto drawGridSquare = [&](int row, int col, uint16_t color) {
        int squareSize = 5;
        int gridPixelWidth = plotAreaWidth / GRID_SIZE;
        int gridPixelHeight = plotAreaHeight / GRID_SIZE;
        int x = marginX + col * gridPixelWidth;
        int y = marginYTop + row * gridPixelHeight;
        tft.drawRect(x, y, squareSize, squareSize, color);
    };

    // Place labels: attempt left side first
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    auto placeLabelAndLine = [&](Label& label, bool tryLeftFirst) -> bool {
        LabelVerticalPlacement verticalPlacement = label.verticalPlacement;
        int textY = calculateVerticalPosition(label.y_initial, label.textHeight, verticalPlacement);
        label.y = textY;

        std::pair<int,int> initialGridPos = pixelToGrid(marginX, label.y, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
        if (initialGridPos.first == -1) return false;

        int startRow = initialGridPos.first;
        bool placed = false;
        int searchRadius = 0;

        while (!placed && searchRadius < GRID_SIZE / 2) {
            int currentRowUp = startRow - searchRadius;
            int currentRowDown = startRow + searchRadius;

            auto tryRow = [&](int row) -> bool {
                if (row < 0 || row >= GRID_SIZE) return false;
                int startColLeft = 0;
                int endColLeft = GRID_SIZE / 2 - 1;
                int startColRight = GRID_SIZE / 2;
                int endColRight = GRID_SIZE - 1;

                auto tryPlacementInRow = [&](int col) -> bool {
                    std::pair<int, int> gridPixelPos = gridToPixel(row, col, plotAreaWidth, plotAreaHeight, marginX, marginYTop);
                    int gridPixelX = gridPixelPos.first;
                    int gridPixelYCenter = gridPixelPos.second;
                    int textX;
                    int newTextY;

                    if (col < GRID_SIZE / 2) {
                        textX = gridPixelX + LABEL_LINE_LENGTH + 2;
                        label.lineStartX = gridPixelX;
                        label.lineEndX = textX - 2;
                        newTextY = gridPixelYCenter;
                    } else {
                        textX = gridPixelX - label.textWidth - LABEL_LINE_LENGTH - 2;
                        label.lineStartX = gridPixelX;
                        label.lineEndX = textX + label.textWidth + 2;
                        newTextY = gridPixelYCenter - label.textHeight / 2;
                    }

                    label.x = textX;
                    label.lineY = gridPixelPos.second;

                    int originalLabelY = label.y;
                    label.y = newTextY;
                    bool canPlace = canPlaceLabel(label, row, col);
                    label.y = originalLabelY;

                    if (canPlace) {
                        // Update text with value at the chosen grid cell center
                        float newDataValue = inverseScaleValue(gridPixelYCenter, label.minValue, label.maxValue);
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << newDataValue;
                        label.text = ss.str();
                        label.y = newTextY;

                        tft.setTextColor(label.color);
                        tft.drawString(label.text.c_str(), textX, newTextY, 1);
                        tft.drawLine(label.lineStartX, label.lineY, label.lineEndX, label.lineY, label.color);

                        markLabelAsPlaced(label, row, col);
                        placed = true;
                        return true;
                    }
                    return false;
                };

                if (tryLeftFirst) {
                    for (int col = startColLeft; col <= endColLeft; ++col) if (tryPlacementInRow(col)) return true;
                    for (int col = endColRight; col >= startColRight; --col) if (tryPlacementInRow(col)) return true;
                } else {
                    for (int col = endColRight; col >= startColRight; --col) if (tryPlacementInRow(col)) return true;
                    for (int col = startColLeft; col <= endColLeft; ++col) if (tryPlacementInRow(col)) return true;
                }
                return false;
            };

            if (tryRow(currentRowUp)) return true;
            if (currentRowDown < GRID_SIZE && currentRowDown != currentRowUp) {
                if (tryRow(currentRowDown)) return true;
            }
            ++searchRadius;
        }

        return false;
    };

    for (auto &label : labels) {
        placeLabelAndLine(label, true);
    }

    // Time axis labels (top ticks)
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(TL_DATUM);
    if (autoscaleX && endTime > startTime) {
        for (int i = 0; i <= 5; ++i) {
            unsigned long timePoint = startTime + (endTime - startTime) * i / 5;
            time_t t = timePoint / 1000;
            struct tm* tm_info = localtime(&t);
            char buffer[6];
            strftime(buffer, sizeof(buffer), "%H:%M", tm_info);
            int x = marginX + plotAreaWidth * i / 5;
            tft.drawLine(x, marginYTop, x, marginYTop - 5, TFT_DARKGREY);
            tft.drawString(buffer, x, marginYTop - 20, 1);
        }
    }

    // Legend (unchanged)
    int legendY = marginYTop + plotAreaHeight + 10;
    int legendX = marginX;
    int colorSize = 9;
    int textOffset = 15;
    tft.setTextDatum(TL_DATUM);
    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_MAGENTA);
    tft.setTextColor(TFT_WHITE);
    tft.drawString("I", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_YELLOW);
    tft.drawString("V", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_DARKGREY);
    tft.drawString("%", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_BLUE);
    tft.drawString("dT", legendX + textOffset, legendY, 1);
    legendX += 40;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_RED);
    tft.drawString("dT/", legendX + textOffset, legendY, 1);

    legendX = marginX;
    legendY += 12;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_ORANGE);
    tft.drawString("iR(L/UL)", legendX + textOffset, legendY, 1);
    legendX += 90;

    tft.fillRect(legendX, legendY, colorSize, colorSize, TFT_CYAN);
    tft.drawString("iR(Pairs)", legendX + textOffset, legendY, 1);
}



//------------------battery charging

uint32_t chargingStartTime = 0 ;
bool chargeBatteryOld() {
    static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    static int chargingDutyCycle = MAX_CHARGE_DUTY_CYCLE; // Initialize with max for the first run
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING");

    // If not currently charging, start the charging process
    if (!isCharging) {
        Serial.println("Starting battery charging process...");
        isCharging = true;
        chargingStartTime = millis(); // Record the start time
        lastChargeEvaluationTime = millis();
        lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE; // Reset to max on new charging start

        // Find the initial optimal charging duty cycle
        chargingDutyCycle = findOptimalChargingDutyCycle(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE);
        lastOptimalDutyCycle = chargingDutyCycle; // Store the initial optimal duty cycle

        // Apply the charging current
        dutyCycle = chargingDutyCycle;
        analogWrite(pwmPin, chargingDutyCycle);

        // Store initial data for monitoring
        MeasurementData initialData;
        getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                              initialData.t1_millivolts, initialData.voltage, initialData.current);
        processThermistorData(initialData, "intial readings");
        Serial.printf("Charging started - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                      chargingDutyCycle, initialData.current, initialData.temp1, initialData.temp2, initialData.tempDiff);

        // Log the initial state
        unsigned long currentTimestamp = millis();
        ChargeLogData logEntry;
        logEntry.timestamp = currentTimestamp;
        logEntry.current = initialData.current;
        logEntry.voltage = initialData.voltage;
        logEntry.ambientTemperature = initialData.temp1; // ambient temperature
        logEntry.batteryTemperature = initialData.temp2; // battery temperature
        logEntry.dutyCycle = chargingDutyCycle;
        logEntry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        logEntry.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(logEntry);
        return true;
    }

// Check for total charging timeout 
    unsigned long currentTime = millis();
    if (currentTime - chargingStartTime >= TOTAL_TIMEOUT) {
        Serial.println("Charging timeout, stopping charging for safety reasons. ");
        // Stop charging
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        isCharging = false;
        return false;
    }

    // Periodically re-evaluate the MH electrode voltage and adjust charging current
    currentTime = millis();
    if (currentTime - lastChargeEvaluationTime > CHARGE_EVALUATION_INTERVAL_MS) {

//check for overcharge from over temperature
    Serial.println("end of charge by temperature delta check...");

    // Get current temperature and voltage readings
    //    float temp1, temp2, tempDiff, t1_millivolts, voltage, current;
    double temp1, temp2, tempDiff;
    float t1_millivolts;
    float voltage;
    float current;

    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);

    float tempRise = estimateTempDiff(voltage, voltage, current, (regressedInternalResistancePairsIntercept), temp1,currentTime,lastChargeEvaluationTime,temp2); // if 0 then no power
  // resistance of MH electrode sums with internal resistance.
    Serial.print("Estimated temperature rise due to Rint heating: ");
    Serial.print(tempRise);
    Serial.println(" °C");
    // Check if temperature difference exceeds threshold - stop charging if it does
    MAX_DIFF_TEMP = (MAX_TEMP_DIFF_THRESHOLD+tempRise); // update max graph to visualize shutdown threshold
    if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD+tempRise)) {
        Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C), stopping charging\n",
                      tempDiff, MAX_TEMP_DIFF_THRESHOLD);

        if (overtemp_trip_counter++ > OVERTEMP_TRIP_TRESHOLD) { // transient ambient temperature changes might trigger overtemperature tripping
                                                                // this mechanism allows repeating the check before tripping. each theshold is one reevaluation period ofcourse
        // Stop charging
        overtemp_trip_counter = 0 ; // reset trip counter
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        isCharging = false;
        return false;
        } // if (overtemp_trip_counter ...
    }

// end of over temperature check

        Serial.println("Re-evaluating charging parameters...");
        // Temporarily pause charging to measure unloaded voltage
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        delay(UNLOADED_VOLTAGE_DELAY_MS);

        // Calculate the starting duty cycle for the next evaluation
        int suggestedStartDutyCycle = min(lastOptimalDutyCycle + (int)(1.0 * lastOptimalDutyCycle), MAX_CHARGE_DUTY_CYCLE); //TODO: - this is all fudge factoring

        // Find the optimal charging duty cycle again, starting from the suggested value
//        chargingDutyCycle = findOptimalChargingDutyCycle(MAX_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle);
        chargingDutyCycle = findOptimalChargingDutyCycle(suggestedStartDutyCycle,MIN_CHARGE_DUTY_CYCLE);

        lastOptimalDutyCycle = chargingDutyCycle; // Update the last optimal duty cycle

        // Apply the updated charging current
        dutyCycle = chargingDutyCycle;
        analogWrite(pwmPin, chargingDutyCycle);

        lastChargeEvaluationTime = currentTime;

        Serial.printf("Charging parameters updated - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                      chargingDutyCycle, current, temp1, temp2, tempDiff);

        // Log the charging parameters after re-evaluation
        unsigned long currentTimestamp = millis();
        ChargeLogData logEntry;
        logEntry.timestamp = currentTimestamp;
        logEntry.current = current;
        logEntry.voltage = voltage;
        // Assuming temp2 is battery temperature and we need a way to get ambient temperature
        logEntry.ambientTemperature = temp1;
        logEntry.batteryTemperature = temp2;
        logEntry.dutyCycle = chargingDutyCycle;
        logEntry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        logEntry.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(logEntry);
    }

    return true; // Charging is still in progress
}

// new chargebattery

// ---------- Integrated non-blocking chargeBattery() ----------
bool chargeBattery() {
      static int lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;
    unsigned long now = millis();

    // Safety: global timeout check (always)
    if (isCharging && (now - chargingStartTime >= TOTAL_TIMEOUT)) {
        Serial.println("Charging timeout, stopping charging for safety reasons.");
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        //isCharging = false;
        chargingState = CHARGE_STOPPED;
        return false;
    }

    // If charging is not started yet, initialize and start async FindOpt to compute initial duty
    if (!isCharging) {
        Serial.println("Starting battery charging process...");
        isCharging = true;
        chargingStartTime = now;
        lastChargeEvaluationTime = now;
        overtemp_trip_counter = 0;
        lastOptimalDutyCycle = MAX_CHARGE_DUTY_CYCLE;

        // initial readings & log
        MeasurementData initialData;
        getThermistorReadings(initialData.temp1, initialData.temp2, initialData.tempDiff,
                              initialData.t1_millivolts, initialData.voltage, initialData.current);
        processThermistorData(initialData, "initial readings");
        Serial.printf("Charging started - Duty Cycle: %d, Current: %.3fA, T1: %.2f°C, T2: %.2f°C, Diff: %.2f°C\n",
                      MAX_CHARGE_DUTY_CYCLE, initialData.current, initialData.temp1, initialData.temp2, initialData.tempDiff);

        ChargeLogData startLog;
        startLog.timestamp = now;
        startLog.current = initialData.current;
        startLog.voltage = initialData.voltage;
        startLog.ambientTemperature = initialData.temp1;
        startLog.batteryTemperature = initialData.temp2;
        startLog.dutyCycle = MAX_CHARGE_DUTY_CYCLE;
        startLog.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        startLog.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(startLog);

        // Begin asynchronous FindOpt manager (this will internally kick off an initial unloaded+highDC measurement)
        startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, MIN_CHARGE_DUTY_CYCLE, false);
        chargingState = CHARGE_FIND_OPT;
        return true;
    }

    // ---------------- Always advance async managers (keeps measurement engine alive)
    // measurementStep() should be cheap and non-blocking (advances timers).
    measurementStep();

    // If the FindOpt manager is active, step it (non-blocking). This also uses measurementStep internally.
    if (/* findOpt.active */ findOptimalChargingDutyCycleStepAsync()) {
        // Still working on finding optimal duty — charging is logically in-progress but no duty applied yet or waiting.
        // Keep calling this function on subsequent handleBatteryCharging() ticks.
        return true; // still charging
    }

    // If we reach here, the every-active FindOpt manager completed earlier and must have stored its result
    // (manager should set findOpt.optimalDC or similar). We'll read that.
    // We only react to a find-opt completion when in the appropriate state:
    if (chargingState == CHARGE_FIND_OPT) {
        // APPLY optimal duty returned by FindOpt manager
        int appliedDC = MIN_CHARGE_DUTY_CYCLE;
        // try to read the optimal from the manager; adapt this access to your manager structure:
//        #ifdef HAS_FINDOPT_STRUCT
            appliedDC = findOpt.optimalDC;
//        #else
//            // if your manager writes a cached var, adapt the name accordingly:
//            appliedDC = cachedOptimalDutyCycle; // fallback if your code keeps this
//        #endif

        // sanitize
        if (appliedDC < MIN_CHARGE_DUTY_CYCLE) appliedDC = MIN_CHARGE_DUTY_CYCLE;
        if (appliedDC > MAX_CHARGE_DUTY_CYCLE) appliedDC = MAX_CHARGE_DUTY_CYCLE;

        dutyCycle = appliedDC;
        analogWrite(pwmPin, dutyCycle);
        lastOptimalDutyCycle = dutyCycle;
        lastChargeEvaluationTime = now;
        chargingState = CHARGE_MONITOR;

        Serial.printf("Applied optimal duty cycle: %d\n", dutyCycle);

        // Log state immediately after applying new duty
        MeasurementData afterApply;
        getThermistorReadings(afterApply.temp1, afterApply.temp2, afterApply.tempDiff,
                              afterApply.t1_millivolts, afterApply.voltage, afterApply.current);
        ChargeLogData entry;
        entry.timestamp = now;
        entry.current = afterApply.current;
        entry.voltage = afterApply.voltage;
        entry.ambientTemperature = afterApply.temp1;
        entry.batteryTemperature = afterApply.temp2;
        entry.dutyCycle = dutyCycle;
        entry.internalResistanceLoadedUnloaded = regressedInternalResistanceIntercept;
        entry.internalResistancePairs = regressedInternalResistancePairsIntercept;
        chargeLog.push_back(entry);

        return true;
    }

    // ---------------- Monitoring state: periodically re-evaluate and check over-temp
    if (chargingState == CHARGE_MONITOR) {
            // read current temps & voltages
            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);
            if (current > maximumCurrent) {
              if (dutyCycle>0){
                dutyCycle--;
                analogWrite(pwmPin,dutyCycle); // clamp to maximum current
              }
            }
      
        // Global safety timeout re-check
        if (now - chargingStartTime >= TOTAL_TIMEOUT) {
            Serial.println("Charging timeout, stopping charging for safety reasons.");
            dutyCycle = 0;
            analogWrite(pwmPin, 0);
            //isCharging = false;
            chargingState = CHARGE_STOPPED;
            return false;
        }

        // When it's time to re-evaluate:
        if (now - lastChargeEvaluationTime >= CHARGE_EVALUATION_INTERVAL_MS) {
            Serial.println("end of charge by temperature delta check...");

            // read current temps & voltages
            double temp1, temp2, tempDiff;
            float t1_mV, voltage, current;
            getThermistorReadings(temp1, temp2, tempDiff, t1_mV, voltage, current);

            // compute estimated temperature rise due to internal heating (keeps original call order)
            float tempRise = estimateTempDiff(voltage, voltage, current,
                                              regressedInternalResistancePairsIntercept,
                                              temp1, now, lastChargeEvaluationTime, temp2); // lastChargeEvaluationTime is previous eval time
            Serial.print("Estimated temperature rise due to Rint heating: ");
            Serial.print(tempRise);
            Serial.println(" °C");

            // update visualization threshold
            MAX_DIFF_TEMP = (MAX_TEMP_DIFF_THRESHOLD + tempRise);

            // over-temp logic (transient allowed a few times via counter)
            if (tempDiff > (MAX_TEMP_DIFF_THRESHOLD + tempRise)) {
                Serial.printf("Temperature difference (%.2f°C) exceeds threshold (%.2f°C).\n", tempDiff, MAX_TEMP_DIFF_THRESHOLD);
                if (overtemp_trip_counter++ >= OVERTEMP_TRIP_TRESHOLD) {
                    // permanent trip -> stop charging
                    overtemp_trip_counter = 0;
                    dutyCycle = 0;
                    analogWrite(pwmPin, 0);
                    //isCharging = false;
                    chargingState = CHARGE_STOPPED;
                    Serial.println("Over-temperature trip: charging stopped.");
                    return false;
                } else {
                    Serial.printf("Over-temp transient: trip counter now %d (will re-check next evaluation).\n", overtemp_trip_counter);
                    // update lastChargeEvaluationTime to avoid immediate re-check; allow a few eval intervals
                    lastChargeEvaluationTime = now;
                    //return true;
                }
            }

            // Not over-temp: prepare to re-evaluate charging params.
            Serial.println("Re-evaluating charging parameters (non-blocking)...");
            // Pause charging output (like original)
            dutyCycle = 0;
            analogWrite(pwmPin, 0);

            // suggested start fudge from original code
            int suggestedStartDutyCycle = min(lastOptimalDutyCycle + (int)(1.0 * lastOptimalDutyCycle),
                                             MAX_CHARGE_DUTY_CYCLE);

            // start async FindOpt manager — it will schedule its internal unloaded+highDC measurement,
            // then binary search measurements, each done non-blocking across subsequent calls to
            // findOptimalChargingDutyCycleStepAsync().
            startFindOptimalManagerAsync(MAX_CHARGE_DUTY_CYCLE, suggestedStartDutyCycle, true);

            // switch state to FindOpt so the above completion path will detect it and apply duty
            chargingState = CHARGE_FIND_OPT;

            // Do NOT set lastChargeEvaluationTime yet — do that once new duty is applied.
            return true;
        }
    }

    // If in CHARGE_STOPPED, ensure outputs are safe
    if (chargingState == CHARGE_STOPPED) {
        dutyCycle = 0;
        analogWrite(pwmPin, 0);
        // Remain stopped until an external start/reset occurs
        return false;
    }

    // Default: charging still in progress
    return isCharging;
}


void startCharging() {
  if (!isCharging) {
    Serial.println("Initiating battery charging...");
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("CHARGING");
    overtemp_trip_counter = 0 ; // reset trip counter
    //isCharging = true;
    chargeBattery();
  } else {
    Serial.println("Charging already in progress");
  }
}

void stopCharging() {
  if (isCharging) {
    Serial.println("Manually stopping charging");
    dutyCycle = 0;
    analogWrite(pwmPin, 0);
    isCharging = false;
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
    tft.printf("STOPPED");
  }
}

void handleBatteryCharging() {
  if (isCharging) {
    // Update charging status and check if it's still charging
    if (!chargeBattery()) {
      // Charging has stopped
      dutyCycle = 0;
      analogWrite(pwmPin, 0);
      isCharging = false;
      tft.setTextColor(TFT_GREEN);
      tft.setTextSize(1);
      tft.setCursor(14*7, PLOT_Y_START + PLOT_HEIGHT + 20);
      tft.printf("COMPLETE");
    }
  }
}

void loop() {
    unsigned long now = millis();

    // ---- 1. Plot log updates and drawing (every 1 s)
    if (now - lastPlotUpdateTime >= PLOT_UPDATE_INTERVAL_MS) {
        lastPlotUpdateTime = now;

        double temp1, temp2, tempDiff;
        float t1_millivolts;
        float voltage;
        float current;

        getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
        printThermistorSerial(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
        updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
        prepareTemperaturePlot();
        plotVoltageData();
        plotTemperatureData();
        displayTemperatureLabels(temp1, temp2, tempDiff, t1_millivolts, voltage, current);

        tft.setTextColor(TFT_WHITE, TFT_BLACK); // with wipe underneath
        tft.setCursor(19 * 10, PLOT_Y_START + PLOT_HEIGHT + 20);
        tft.print(mAh_charged, 3);
        tft.print(" mAh");
    }

    // ---- 2. Charging housekeeping (every 100 ms)
    if (now - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
        lastChargingHouseTime = now;
        handleBatteryCharging();
    }

    // ---- 3. IR remote handling (every 200 ms)
    if (now - lastIRHandleTime >= IR_HANDLE_INTERVAL_MS) {
        lastIRHandleTime = now;
        if (IrReceiver.decode()) {
            handleIRCommand();
            IrReceiver.resume();
        }
    }

    // place background tasks below
    
}
void handleIRCommand() {

      if (IrReceiver.decodedIRData.protocol == SAMSUNG) {
            if (IrReceiver.decodedIRData.address == 0x7) {
      //debug
            Serial.print(F("Command 0x"));
            Serial.println(IrReceiver.decodedIRData.command, HEX);
        switch(IrReceiver.decodedIRData.command) {
        case RemoteKeys::KEY_PLAY:{
        isMeasuringResistance = true;
        measureInternalResistance();
        break;
        }
        case RemoteKeys::KEY_INFO:{
          if (!isMeasuringResistance) {
          displayInternalResistanceGraph(); // Display the internal resistance graph after measurement
          delay(15000); // wait 10 seconds
        }
        break;
        }

        case RemoteKeys::KEY_SOURCE:{
          drawChargePlot(true, true);// Display the charging process log graph
          delay(20000); // wait 20 seconds
          tft.fillScreen(TFT_BLACK); // clear junk afterwards
        break;
        }

#ifdef DEBUG_LABELS
        case RemoteKeys::KEY_0:{
          testGraph();
          delay(20000); // wait 20 seconds
          tft.fillScreen(TFT_BLACK); // clear junk afterwards
        break;
        }
#endif

        case RemoteKeys::KEY_POWER:{
        resetAh = true; // reset mAh counter
        buildCurrentModel(false); // Build the model from scratch
        startCharging(); // slow charge battery
        break;
        }

        }// switch
            }//address 7
      }//samsung
}// handle IR
