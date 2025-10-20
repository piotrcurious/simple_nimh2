#ifndef CONFIG_H
#define CONFIG_H

// --- Hardware Pin Definitions ---
#define IR_RECEIVE_PIN 2
#define PWM_PIN 10
#define THERMISTOR_PIN_1 4
#define THERMISTOR_VCC_PIN 17
#define CURRENT_SHUNT_PIN 5
#define VOLTAGE_READ_PIN 6

// --- PWM Configuration ---
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION_BITS 8
#define MAX_DUTY_CYCLE ((1 << PWM_RESOLUTION_BITS) - 1)

// --- ADC Configuration ---
#define CURRENT_SHUNT_ATTENUATION ADC_11db
#define CURRENT_SHUNT_OVERSAMPLING 256
#define VOLTAGE_ATTENUATION ADC_11db
#define VOLTAGE_OVERSAMPLING 256

// --- Thermistor & Temperature Sensor Configuration ---
#define THERMISTOR_NOMINAL_RESISTANCE 10000
#define THERMISTOR_BETA_COEFFICIENT 3950
#define THERMISTOR_SERIES_RESISTOR 10000
#define NOMINAL_TEMPERATURE 25.0

// --- Current & Voltage Measurement Configuration ---
#define CURRENT_SHUNT_RESISTANCE 0.05
#define MAIN_VCC_RATIO 0.50
#define CURRENT_SHUNT_PIN_ZERO_OFFSET 0.0
#define MEASURABLE_CURRENT_THRESHOLD 0.010 // 10mA

// --- Application Timing and Delays (in milliseconds) ---
#define PLOT_UPDATE_INTERVAL_MS 1000
#define PLOT_DATA_UPDATE_INTERVAL 500
#define IR_HANDLE_INTERVAL_MS 100
#define CHARGING_HOUSEKEEP_INTERVAL 1000
#define BUILD_CURRENT_MODEL_DELAY 2000
#define IR_STABILIZATION_DELAY 5000
#define IR_UNLOADED_DELAY 5000

// --- Plotting & Display Configuration ---
#define PLOT_WIDTH 320
#define PLOT_HEIGHT (216 - 3)
#define PLOT_X_START 0
#define PLOT_Y_START 0
#define MAX_DATA_POINTS PLOT_WIDTH

#endif // CONFIG_H