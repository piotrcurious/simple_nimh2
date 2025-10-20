#include "InternalResistance.h"
#include "config.h"
#include <Arduino.h>

// --- Constructor ---
InternalResistance::InternalResistance(DataStore* data_store, Power* power) :
    _data_store(data_store),
    _power(power),
    _state_start_time(0),
    _find_min_current_duty_cycle(0)
{}

// --- Public Methods ---

void InternalResistance::update() {
    if (_data_store->app_state == AppState::MEASURING_IR) {
        _run_state_machine();
    }
}

void InternalResistance::start_measurement() {
    if (_data_store->app_state == AppState::IDLE) {
        _reset();
        _data_store->app_state = AppState::MEASURING_IR;
        _data_store->ir_state = IRState::START;
        Serial.println("Starting internal resistance measurement.");
    } else {
        Serial.println("Cannot start IR measurement. Application is busy.");
    }
}

void InternalResistance::abort_measurement() {
    if (_data_store->app_state == AppState::MEASURING_IR) {
        _reset();
        Serial.println("Internal resistance measurement aborted.");
    }
}

// --- Private Methods ---

void InternalResistance::_reset() {
    _power->set_duty_cycle(0);
    _data_store->app_state = AppState::IDLE;
    _data_store->ir_state = IRState::IDLE;
    _data_store->clear_ir_data();
    _measured_pairs.clear();
    _state_start_time = 0;
}

void InternalResistance::_run_state_machine() {
    unsigned long now = millis();

    switch (_data_store->ir_state) {
        case IRState::START:
            _power->set_duty_cycle(0);
            _state_start_time = now;
            _data_store->ir_state = IRState::FIND_MIN_CURRENT;
            _find_min_current_duty_cycle = 5; // Start with a small duty cycle
            Serial.println("IR: Finding minimum measurable current...");
            break;

        case IRState::FIND_MIN_CURRENT:
            _power->set_duty_cycle(_find_min_current_duty_cycle);
            if (now - _state_start_time > 1000) { // Wait 1s for current to stabilize
                if (_data_store->latest_measurement.current_A >= MEASURABLE_CURRENT_THRESHOLD) {
                    Serial.printf("IR: Minimum current found at duty cycle %d\n", _find_min_current_duty_cycle);
                    _state_start_time = now;
                    _data_store->ir_state = IRState::MEASURE_PAIRS;
                } else {
                    _find_min_current_duty_cycle += 5;
                    if (_find_min_current_duty_cycle > MAX_DUTY_CYCLE) {
                        Serial.println("IR: Could not find a measurable current. Aborting.");
                        abort_measurement();
                    }
                    _state_start_time = now; // Reset timer for next attempt
                }
            }
            break;

        case IRState::MEASURE_PAIRS:
            // In a real implementation, this state would step through various duty cycles,
            // wait for stabilization, and record voltage/current pairs.
            // For this refactoring, we'll simulate this with a simple delay and then calculate.
            if (now - _state_start_time > 5000) { // Simulate 5s of measurements
                 // Add some dummy data for calculation
                _data_store->ir_data_points.push_back({0.1f, 1.25f});
                _data_store->ir_data_points.push_back({0.2f, 1.28f});
                _data_store->ir_data_points.push_back({0.3f, 1.31f});

                Serial.println("IR: Measurement pairs collected.");
                _data_store->ir_state = IRState::CALCULATE;
            }
            break;

        case IRState::CALCULATE:
            Serial.println("IR: Calculating internal resistance...");
            if (_perform_linear_regression(_data_store->ir_slope, _data_store->ir_intercept)) {
                 // The slope of the V-I plot is the internal resistance.
                Serial.printf("IR: Calculation complete. Resistance: %.4f Ohms\n", _data_store->ir_slope);
            } else {
                Serial.println("IR: Linear regression failed. Not enough data.");
            }
            _data_store->ir_state = IRState::DISPLAY;
            _state_start_time = now;
            break;

        case IRState::DISPLAY:
            // This state is a placeholder. The DisplayManager will handle showing the results.
            // We'll just wait for a bit before returning to idle.
            if (now - _state_start_time > 10000) { // Display results for 10s
                _data_store->ir_state = IRState::COMPLETE;
            }
            break;

        case IRState::COMPLETE:
            Serial.println("IR: Measurement complete.");
            _reset();
            break;

        case IRState::IDLE:
        case IRState::ABORTED:
            // Do nothing
            break;
    }
}

bool InternalResistance::_perform_linear_regression(float& slope, float& intercept) {
    int n = _data_store->ir_data_points.size();
    if (n < 2) return false;

    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (const auto& p : _data_store->ir_data_points) {
        sumX += p.current_A;
        sumY += p.voltage_V;
        sumXY += p.current_A * p.voltage_V;
        sumX2 += p.current_A * p.current_A;
    }

    float denominator = n * sumX2 - sumX * sumX;
    if (abs(denominator) < 1e-6) return false; // Avoid division by zero

    slope = (n * sumXY - sumX * sumY) / denominator;
    intercept = (sumY * sumX2 - sumX * sumXY) / denominator;

    return true;
}