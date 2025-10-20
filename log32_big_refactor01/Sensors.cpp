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

void Sensors::begin() {
    _sht4x_sensor.begin();
    _thermistor_sensor.begin();
    _last_mah_update_time = millis();
}

// Main method to read from all sensors and update the DataStore.
void Sensors::read() {
    _read_and_update_all_sensors();
    _update_mah_charged();

    // The _read_and_update_all_sensors method already populates the
    // latest_measurement struct in the data_store. Now, add it to history.
    _data_store->add_history_point(_data_store->latest_measurement);

    // Delay to control the sensor read frequency
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

// --- Private Helper Methods ---

void Sensors::_read_and_update_all_sensors() {
    _sht4x_sensor.read();
    while (_sht4x_sensor.isLocked()) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    };
    _thermistor_sensor.read(_sht4x_sensor.getTemperature());

    // Use functions from analog.cpp to get current and voltage
    float current_ma = readCurrent();

    uint32_t current_time = millis();
    if ((current_time - _last_voltage_update_time) > 250) {
        float voltage_mv = readVoltage(_thermistor_sensor.getVCC());
        _data_store->latest_measurement.voltage_V = voltage_mv / 1000.0f;
        _last_voltage_update_time = current_time;
    }

    // Populate the measurement struct in DataStore
    _data_store->latest_measurement.timestamp_ms = current_time;
    // Duty cycle is not set here; it's set by the Power class
    _data_store->latest_measurement.temp_ambient_C = _sht4x_sensor.getTemperature();
    _data_store->latest_measurement.temp_surface_C = _thermistor_sensor.getTemperature2();
    _data_store->latest_measurement.temp_diff_C = _thermistor_sensor.getDifference();
    _data_store->latest_measurement.current_A = current_ma / 1000.0f;
}

void Sensors::_update_mah_charged() {
    uint32_t current_time = millis();
    uint32_t time_elapsed_ms = current_time - _last_mah_update_time;
    if (time_elapsed_ms == 0) return;

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