#include "Power.h"
#include "config.h"
#include <Arduino.h>

// --- Constructor ---
Power::Power(DataStore* data_store) :
    _data_store(data_store),
    _build_model_step(0),
    _build_model_duty_cycle(0),
    _build_model_last_step_time(0),
    _last_charging_housekeeping_time(0)
{}

// --- Public Methods ---

void Power::begin() {
    // Correctly initialize PWM with pin and resolution
    analogWriteResolution(PWM_PIN, PWM_RESOLUTION_BITS);
    analogWriteFrequency(PWM_PIN, PWM_FREQUENCY);
    pinMode(PWM_PIN, OUTPUT);
    set_duty_cycle(0);
}

void Power::update() {
    unsigned long now = millis();
    switch (_data_store->app_state) {
        case AppState::BUILDING_MODEL:
            _build_current_model_step();
            break;
        case AppState::CHARGING:
            if (now - _last_charging_housekeeping_time >= CHARGING_HOUSEKEEP_INTERVAL) {
                _last_charging_housekeeping_time = now;
                if (!_charge_battery()) {
                    stop_charging();
                }
            }
            break;
        default:
            // Do nothing
            break;
    }
}

void Power::set_duty_cycle(int duty_cycle) {
    int clamped_duty = constrain(duty_cycle, 0, MAX_DUTY_CYCLE);
    _data_store->latest_measurement.duty_cycle = clamped_duty;
    analogWrite(PWM_PIN, clamped_duty);
}

void Power::start_charging() {
    if (_data_store->current_model.is_built) {
        _data_store->app_state = AppState::CHARGING;
        Serial.println("Starting charge cycle.");
    } else {
        Serial.println("Current model not built. Building model first...");
        start_model_build();
    }
}

void Power::stop_charging() {
    set_duty_cycle(0);
    _data_store->app_state = AppState::IDLE;
    Serial.println("Charging stopped.");
}

void Power::start_model_build() {
    _data_store->app_state = AppState::BUILDING_MODEL;
    _build_model_step = 0; // Reset state machine
    Serial.println("Starting current model build process.");
}

// --- Private Helper Methods ---

void Power::_build_current_model_step() {
    unsigned long now = millis();

    if (_build_model_step == 0) { // Initialization
        Serial.println("Starting fresh model building.");
        _model_duty_cycles.clear();
        _model_currents.clear();
        _model_duty_cycles.push_back(0.0f);
        _model_currents.push_back(0.0f);
        _build_model_duty_cycle = 5; // Start at a low duty cycle
        _build_model_step = 1;
    }

    if (_build_model_step == 1) { // Set duty cycle and wait
        if (_build_model_duty_cycle <= MAX_DUTY_CYCLE) {
            set_duty_cycle(_build_model_duty_cycle);
            _build_model_last_step_time = now;
            _build_model_step = 2;
        } else {
            _build_model_step = 3; // Finished collecting data
        }
    }

    if (_build_model_step == 2) { // Wait for stabilization and measure
        if (now - _build_model_last_step_time >= BUILD_CURRENT_MODEL_DELAY) {
            Measurement data = _data_store->latest_measurement;
            if (data.current_A >= MEASURABLE_CURRENT_THRESHOLD) {
                _model_duty_cycles.push_back(static_cast<float>(_build_model_duty_cycle));
                _model_currents.push_back(data.current_A);
                 Serial.printf("Model point: %d, %.3fA\n", _build_model_duty_cycle, data.current_A);
            }
            _build_model_duty_cycle += 5;
            _build_model_step = 1; // Go to next duty cycle
        }
    }

    if (_build_model_step == 3) { // Calculate model
        if (_model_duty_cycles.size() < 2) {
            Serial.println("Not enough data points to build a reliable model.");
            _data_store->current_model.is_built = false;
            stop_charging();
            return;
        }

        int degree = 3;
        int numPoints = _model_duty_cycles.size();
        Eigen::MatrixXd A(numPoints, degree + 1);
        Eigen::VectorXd b(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            for (int j = 0; j <= degree; ++j) {
                A(i, j) = std::pow(_model_duty_cycles[i], j);
            }
            b(i) = _model_currents[i];
        }

        Eigen::VectorXd coeffs = A.householderQr().solve(b);
        if (degree >= 0) coeffs(0) = 0.0f;

        _data_store->current_model.coefficients = coeffs;
        _data_store->current_model.is_built = true;

        Serial.println("Current model built successfully.");
        set_duty_cycle(0);
        _data_store->app_state = AppState::IDLE;
        start_charging(); // Automatically start charging after model is built
        _build_model_step = 0; // Reset for next time
    }
}

bool Power::_charge_battery() {
    // This is a placeholder for the charging logic.
    set_duty_cycle(128); // Example: 50% duty cycle
    return true;
}

float Power::_estimate_current(int duty_cycle) {
    if (!_data_store->current_model.is_built) return 0.0f;

    float estimated_current = 0.0f;
    float duty_cycle_float = static_cast<float>(duty_cycle);
    for (int i = 0; i < _data_store->current_model.coefficients.size(); ++i) {
        estimated_current += _data_store->current_model.coefficients(i) * std::pow(duty_cycle_float, i);
    }
    return std::max(0.0f, estimated_current);
}