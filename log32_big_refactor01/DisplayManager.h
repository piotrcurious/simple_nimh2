#ifndef DISPLAYMANAGER_H
#define DISPLAYMANAGER_H

#include "DataStore.h"
#include <TFT_eSPI.h>

// The DisplayManager class is responsible for all drawing operations on the
// TFT screen. It reads data from the DataStore and updates the display
// based on the current DisplayState.
class DisplayManager {
public:
    // --- Constructor ---
    DisplayManager(DataStore* data_store);

    // --- Public Methods ---
    void begin();   // Initializes the TFT screen.
    void update();  // Main update loop, redraws the screen as needed.
    void set_display_state(DisplayState state); // Switches the active display.

private:
    // --- Private Members ---
    DataStore* _data_store; // Pointer to the central data repository.
    TFT_eSPI _tft;          // The TFT screen object.
    unsigned long _last_plot_update_time;

    // --- Private Drawing Methods ---
    void _draw_main_display();
    void _draw_charge_graph();
    void _draw_ir_graph();

    void _draw_temperature_plot();
    void _draw_voltage_plot();
    void _display_temperature_labels();
    void _display_main_labels();
};

#endif // DISPLAYMANAGER_H