#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>

#include "dummy_esp8266_client.h"

// Forward definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define WEBSERVER_H

#include "../max6675_client_display_esp8266.ino"

MockSerial Serial;
unsigned long mock_millis = 0;
uint8_t mock_eeprom_cells[32] = {0};
int mock_button_pin_val = HIGH;

void test_eeprom_save_restore() {
    std::cout << "Testing ESP8266 client display EEPROM save/restore state..." << std::endl;

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
    std::cout << "Testing ESP8266 client display button press screen mode cycling..." << std::endl;

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

void test_interoperability_payload_parsing() {
    std::cout << "Testing ESP8266 full server-to-client JSON payload parsing interoperability..." << std::endl;

    // Create a mock payload matching the server's broadcast format exactly
    std::string server_payload = "{\"t1\":345.50,\"t2\":120.75,\"t3\":670.00,\"t4\":890.25,\"e1\":0,\"e2\":0,\"e3\":0,\"e4\":1}";

    // Reset client arrays
    for (int i = 0; i < 4; i++) {
        temps[i] = 0.0f;
        errors[i] = true;
    }

    // Invoke client's websocket handler
    webSocketEvent(WStype_TEXT, (uint8_t*)server_payload.c_str(), server_payload.length());

    std::cout << "  Parsed values -> T1: " << temps[0] << ", T2: " << temps[1] << ", T3: " << temps[2] << ", T4: " << temps[3] << std::endl;
    std::cout << "  Error statuses -> E1: " << errors[0] << ", E2: " << errors[1] << ", E3: " << errors[2] << ", E4: " << errors[3] << std::endl;

    // Asserts
    assert(temps[0] == 345.50f);
    assert(temps[1] == 120.75f);
    assert(temps[2] == 670.00f);
    assert(temps[3] == 890.25f);

    assert(!errors[0]);
    assert(!errors[1]);
    assert(!errors[2]);
    assert(errors[3]); // E4 is true

    std::cout << "test_interoperability_payload_parsing PASSED" << std::endl << std::endl;
}

int main() {
    test_eeprom_save_restore();
    test_button_press_screen_cycling();
    test_interoperability_payload_parsing();
    std::cout << "All ESP8266 client display simulation tests PASSED successfully!" << std::endl;
    return 0;
}
