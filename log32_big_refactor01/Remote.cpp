#include "Remote.h"

extern volatile bool resetAh;
extern unsigned long displayStateChangeTime;

Remote::Remote(DataStore& data) : data(data) {}

void Remote::begin() {
    IrReceiver.begin(IR_RECEIVE_PIN);
}

void Remote::handle() {
    portMUX_TYPE ir_mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&ir_mux);
    bool is_ir_data = IrReceiver.decode();
    portEXIT_CRITICAL(&ir_mux);
    if (is_ir_data) {
        handleIRCommand();
        IrReceiver.resume();
    }
}

void Remote::handleIRCommand() {
    if (IrReceiver.decodedIRData.protocol == SAMSUNG && IrReceiver.decodedIRData.address == 0x7) {
        Serial.print(F("Command 0x"));
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        switch(IrReceiver.decodedIRData.command) {
            case RemoteKeys::KEY_PLAY:
                data.currentAppState = APP_STATE_MEASURING_IR;
                currentIRState = IR_STATE_START;
                break;
            case RemoteKeys::KEY_INFO:
                 if (data.currentAppState == APP_STATE_IDLE || data.currentAppState == APP_STATE_CHARGING ) {
                    data.currentDisplayState = DISPLAY_STATE_IR_GRAPH;
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_SOURCE:
                if (data.currentDisplayState != DISPLAY_STATE_CHARGE_GRAPH) {
                    data.currentDisplayState = DISPLAY_STATE_CHARGE_GRAPH;
                    displayStateChangeTime = millis();
                }
                break;
            case RemoteKeys::KEY_POWER:
                resetAh = true;
                data.currentAppState = APP_STATE_BUILDING_MODEL;
                break;

#ifdef DEBUG_LABELS

                case RemoteKeys::KEY_0:{
                // testGraph();
                // delay(20000); // wait 20 seconds
                // tft.fillScreen(TFT_BLACK); // clear junk afterwards
                break;
                }
#endif // #ifdef DEBUG_LABELS
        }
    }
}