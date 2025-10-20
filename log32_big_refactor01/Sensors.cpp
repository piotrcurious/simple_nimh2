#include "Sensors.h"
#include "config.h"
#include <Arduino.h>

// --- Constructor ---
Sensors::Sensors(DataStore* data_store) :
    _data_store(data_store),
    _sht4x_sensor(),
    _thermistor_sensor(THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, 0.0),
    _last_voltage_update_time(0),
    _last_mah_update_time(0)
{}

// --- Public Methods ---

// Initializes the hardware for all sensors.
void Sensors::begin() {
    _sht4x_sensor.begin();
    _thermistor_sensor.begin();

    // Configure ADC for voltage and current readings
    analogReadResolution(12);
    pinMode(CURRENT_SHUNT_PIN, INPUT);
    pinMode(VOLTAGE_READ_PIN, INPUT);

    _last_mah_update_time = millis();
}

// Main method to read from all sensors and update the DataStore.
// This is designed to be called repeatedly from a FreeRTOS task.
void Sensors::read() {
    _read_sht4x();

    // Wait if the SHT4x sensor is busy
    while (_sht4x_sensor.isLocked()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    _read_thermistor();
    _read_voltage_and_current();
    _update_mah_charged();

    // Create a new measurement object with the latest data
    Measurement new_measurement;
    new_measurement.timestamp_ms = millis();
    new_measurement.duty_cycle = _data_store->latest_measurement.duty_cycle; // Persist duty cycle
    new_measurement.temp_ambient_C = _sht4x_sensor.getTemperature();
    new_measurement.temp_surface_C = _thermistor_sensor.getTemperature2();
    new_measurement.temp_diff_C = _thermistor_sensor.getDifference();
    new_measurement.voltage_V = _data_store->latest_measurement.voltage_V;
    new_measurement.current_A = _data_store->latest_measurement.current_A;

    // Update the central data store
    _data_store->latest_measurement = new_measurement;
    _data_store->add_history_point(new_measurement);

    // Delay to control the sensor read frequency
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

// --- Private Helper Methods ---

void Sensors::_read_sht4x() {
    _sht4x_sensor.read();
}

void Sensors::_read_thermistor() {
    // The thermistor reading depends on the ambient temp from SHT4x
    _thermistor_sensor.read(_sht4x_sensor.getTemperature());
}

void Sensors::_read_voltage_and_current() {
    // Read current
    double sum_analog_current = 0;
    for (int i = 0; i < CURRENT_SHUNT_OVERSAMPLING; ++i) {
        sum_analog_current += analogRead(CURRENT_SHUNT_PIN);
    }
    double avg_analog_current = sum_analog_current / CURRENT_SHUNT_OVERSAMPLING;
    double voltage_across_shunt = (avg_analog_current / 4095.0) * 3300.0; // Convert to mV
    _data_store->latest_measurement.current_A = (voltage_across_shunt / CURRENT_SHUNT_RESISTANCE) / 1000.0;

    // Read voltage (less frequently)
    if (millis() - _last_voltage_update_time > 250) {
        double sum_analog_voltage = 0;
        for (int i = 0; i < VOLTAGE_OVERSAMPLING; ++i) {
            sum_analog_voltage += analogRead(VOLTAGE_READ_PIN);
        }
        double avg_analog_voltage = sum_analog_voltage / VOLTAGE_OVERSAMPLING;
        double voltage_mv = (avg_analog_voltage / 4095.0) * 3300.0;
        _data_store->latest_measurement.voltage_V = (voltage_mv * MAIN_VCC_RATIO) / 1000.0;
        _last_voltage_update_time = millis();
    }
}

void Sensors::_update_mah_charged() {
    uint32_t current_time = millis();
    uint32_t time_elapsed_ms = current_time - _last_mah_update_time;
    double time_elapsed_h = (double)time_elapsed_ms / (1000.0 * 3600.0);

    double current_for_mah_calc_ma = _data_store->latest_measurement.current_A * 1000.0;

    // Use the current model to estimate if the real reading is below the measurable threshold
    if (_data_store->current_model.is_built && _data_store->latest_measurement.current_A < MEASURABLE_CURRENT_THRESHOLD) {
        float estimated_current = 0.0f;
        float duty_cycle_float = static_cast<float>(_data_store->latest_measurement.duty_cycle);
        for (int i = 0; i < _data_store->current_model.coefficients.size(); ++i) {
            estimated_current += _data_store->current_model.coefficients(i) * std::pow(duty_cycle_float, i);
        }
        current_for_mah_calc_ma = static_cast<double>(std::max(0.0f, estimated_current) * 1000.0);
    }

    _data_store->mAh_charged += current_for_mah_calc_ma * time_elapsed_h;
    _last_mah_update_time = current_time;
}