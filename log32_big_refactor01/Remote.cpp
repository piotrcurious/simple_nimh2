#include "Remote.h"
#include "config.h"

// --- Constructor ---
Remote::Remote(DataStore* data_store, Power* power, InternalResistance* ir_tester, DisplayManager* display_manager) :
    _data_store(data_store),
    _power(power),
    _ir_tester(ir_tester),
    _display_manager(display_manager),
    _last_ir_check_time(0)
{}

// --- Public Methods ---

void Remote::handle() {
    unsigned long now = millis();
    if (now - _last_ir_check_time < IR_HANDLE_INTERVAL_MS) {
        return;
    }
    _last_ir_check_time = now;

    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x7) {
            _process_command(IrReceiver.decodedIRData.command);
        }
        IrReceiver.resume(); // Prepare for the next signal
    }
}

// --- Private Helper Methods ---

void Remote::_process_command(uint16_t command) {
    Serial.printf("Received IR command: 0x%X\n", command);

    switch (command) {
        case RemoteKeys::KEY_POWER:
            _data_store->reset_mAh();
            _power->start_model_build(); // This will then chain to start_charging()
            break;

        case RemoteKeys::KEY_PLAY:
            _ir_tester->start_measurement();
            break;

        case RemoteKeys::KEY_INFO:
            if (_data_store->app_state == AppState::IDLE || _data_store->app_state == AppState::CHARGING) {
                _display_manager->set_display_state(DisplayState::IR_GRAPH);
            }
            break;

        case RemoteKeys::KEY_SOURCE:
            _display_manager->set_display_state(DisplayState::CHARGE_GRAPH);
            break;

        // Add other cases as needed
        default:
            Serial.println("Unknown command.");
            break;
    }
}