#ifndef REMOTE_H
#define REMOTE_H

#include "DataStore.h"
#include "Power.h"
#include "InternalResistance.h"
#include "DisplayManager.h"

// The Remote class is responsible for handling all input from the
// IR remote control. It decodes the received commands and triggers
// actions in the appropriate modules.
class Remote {
public:
    // --- Constructor ---
    Remote(DataStore* data_store, Power* power, InternalResistance* ir_tester, DisplayManager* display_manager);

    // --- Public Methods ---
    void handle(); // Polls the IR receiver and processes commands.

private:
    // --- Private Members ---
    DataStore* _data_store;
    Power* _power;
    InternalResistance* _ir_tester;
    DisplayManager* _display_manager;
    unsigned long _last_ir_check_time;

    // --- Private Helper Methods ---
    void _process_command(uint16_t command);
};

#endif // REMOTE_H