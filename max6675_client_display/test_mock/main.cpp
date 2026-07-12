#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>

#include "dummy_esp32_client.h"

// Forward definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define WEBSERVER_H

#include "../max6675_client_display.ino"

MockSerial Serial;
unsigned long mock_millis = 0;
uint8_t mock_eeprom_cells[32] = {0};
int mock_button_pin_val = HIGH;

void test_eeprom_save_restore() {
    std::cout << "Testing client display EEPROM save/restore state..." << std::endl;

    // Reset EEPROM cell to default
    mock_eeprom_cells[0] = 2; //SCREEN_COMPOUND_GRAPH
    ScreenMode mode = restoreScreenMode();
    assert(mode == SCREEN_COMPOUND_GRAPH);

    saveScreenMode(SCREEN_RELATIVE_BARS);
    mock_eeprom_cells[0] = (uint8_t)SCREEN_RELATIVE_BARS;
    mode = restoreScreenMode();
    assert(mode == SCREEN_RELATIVE_BARS);

    std::cout << "test_eeprom_save_restore PASSED" << std::endl << std::endl;
}

void test_button_press_screen_cycling() {
    std::cout << "Testing client display button press screen mode cycling..." << std::endl;

    currentScreen = SCREEN_RELATIVE_BARS;
    mock_button_pin_val = HIGH; // Unpressed
    mock_millis = 0;

    // Call loop multiple times with button unpressed
    for (int i = 0; i < 50; i++) {
        mock_millis += 10;
        loop();
    }
    assert(currentScreen == SCREEN_RELATIVE_BARS);

    // Simulate Press Button (LOW) and hold for 100ms
    mock_button_pin_val = LOW;
    for (int i = 0; i < 15; i++) {
        mock_millis += 10;
        loop();
    }

    // Simulate Release Button (HIGH)
    mock_button_pin_val = HIGH;
    for (int i = 0; i < 20; i++) {
        mock_millis += 10;
        loop();
    }

    std::cout << "  Active Screen after button press: " << (int)currentScreen << std::endl;
    assert(currentScreen == SCREEN_ABSOLUTE_BARS); // Cyclied to Absolute bars!

    std::cout << "test_button_press_screen_cycling PASSED" << std::endl << std::endl;
}

int main() {
    test_eeprom_save_restore();
    test_button_press_screen_cycling();
    std::cout << "All local MAX6675 client display simulation tests PASSED successfully!" << std::endl;
    return 0;
}
