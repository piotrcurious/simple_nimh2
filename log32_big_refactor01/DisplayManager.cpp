#include "DisplayManager.h"
#include "config.h"

// --- Constructor ---
DisplayManager::DisplayManager(DataStore* data_store) :
    _data_store(data_store),
    _tft(TFT_eSPI()),
    _last_plot_update_time(0)
{}

// --- Public Methods ---

void DisplayManager::begin() {
    _tft.init();
    _tft.setRotation(1);
    _tft.fillScreen(TFT_BLACK);
}

void DisplayManager::update() {
    unsigned long now = millis();

    // Check if it's time to switch back to the main display
    if (_data_store->display_state != DisplayState::MAIN && now - _data_store->display_state_change_time > 20000) {
        set_display_state(DisplayState::MAIN);
    }

    // Only update the plot at a fixed interval
    if (now - _last_plot_update_time < PLOT_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_plot_update_time = now;

    switch (_data_store->display_state) {
        case DisplayState::MAIN:
            _draw_main_display();
            break;
        case DisplayState::CHARGE_GRAPH:
            _draw_charge_graph();
            break;
        case DisplayState::IR_GRAPH:
            _draw_ir_graph();
            break;
    }
}

void DisplayManager::set_display_state(DisplayState state) {
    if (_data_store->display_state != state) {
        _data_store->display_state = state;
        _data_store->display_state_change_time = millis();
        _tft.fillScreen(TFT_BLACK); // Clear screen on state change
    }
}

// --- Private Drawing Methods ---

void DisplayManager::_draw_main_display() {
    _draw_temperature_plot();
    _draw_voltage_plot();
    _draw_current_plot();
    _display_temperature_labels();
    _display_main_labels();
}

void DisplayManager::_draw_charge_graph() {
    // Placeholder for detailed charge graph logic
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    _tft.setCursor(20, 20);
    _tft.setTextSize(2);
    _tft.println("Charge-Discharge Curve");
    // In a real implementation, this would plot voltage vs. capacity
}

void DisplayManager::_draw_ir_graph() {
    // Placeholder for internal resistance graph logic
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    _tft.setCursor(20, 20);
    _tft.setTextSize(2);
    _tft.println("Internal Resistance (V-I Plot)");

    if (_data_store->ir_data_points.size() >= 2) {
        // Draw axes, points, and the regression line
        char buffer[50];
        sprintf(buffer, "R_int: %.4f Ohms", _data_store->ir_slope);
        _tft.setCursor(20, 50);
        _tft.println(buffer);
    } else {
        _tft.setCursor(20, 50);
        _tft.println("Not enough data.");
    }
}

void DisplayManager::_draw_temperature_plot() {
    // Simplified plotting logic
    for (int i = 1; i < _data_store->temp_surface_history.size(); ++i) {
        _tft.drawLine(i - 1, 100 - _data_store->temp_surface_history[i-1], i, 100 - _data_store->temp_surface_history[i], TFT_RED);
        _tft.drawLine(i - 1, 100 - _data_store->temp_ambient_history[i-1], i, 100 - _data_store->temp_ambient_history[i], TFT_BLUE);
    }
}

void DisplayManager::_draw_voltage_plot() {
    // Simplified plotting logic
     for (int i = 1; i < _data_store->voltage_history.size(); ++i) {
        _tft.drawLine(i - 1, 150 - (_data_store->voltage_history[i-1] * 50), i, 150 - (_data_store->voltage_history[i] * 50), TFT_YELLOW);
    }
}

void DisplayManager::_draw_current_plot() {
    // Simplified plotting logic
     for (int i = 1; i < _data_store->current_history.size(); ++i) {
        _tft.drawLine(i - 1, 200 - (_data_store->current_history[i-1] * 100), i, 200 - (_data_store->current_history[i] * 100), TFT_MAGENTA);
    }
}

void DisplayManager::_display_temperature_labels() {
    char buffer[50];
    const Measurement& latest = _data_store->latest_measurement;

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    _tft.setTextSize(1);

    sprintf(buffer, "Surf: %.2fC Amb: %.2fC Diff: %.2fC", latest.temp_surface_C, latest.temp_ambient_C, latest.temp_diff_C);
    _tft.setCursor(10, 220);
    _tft.print(buffer);
}

void DisplayManager::_display_main_labels() {
    char buffer[50];
    const Measurement& latest = _data_store->latest_measurement;

    _tft.setTextColor(TFT_WHITE, TFT_BLACK);
    _tft.setTextSize(2);

    sprintf(buffer, "V: %.3fV", latest.voltage_V);
    _tft.setCursor(10, 10);
    _tft.print(buffer);

    sprintf(buffer, "I: %.3fA", latest.current_A);
    _tft.setCursor(10, 30);
    _tft.print(buffer);

    sprintf(buffer, "%.3f mAh", _data_store->mAh_charged);
    _tft.setCursor(180, 10);
    _tft.print(buffer);
}