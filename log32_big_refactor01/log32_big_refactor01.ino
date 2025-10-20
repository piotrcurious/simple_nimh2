/*
 * --------------------------------------------------------------------
 * Main sketch file for the Ni-Cd/Ni-MH Battery Charger.
 *
 * This file contains the main setup and loop functions for the Arduino.
 * It initializes all the hardware and software components and then
 * enters the main loop, which drives the application's state machines
 * and handles user input.
 *
 * The application is architected around a set of classes, each
 * responsible for a specific part of the functionality:
 *
 * - DataStore:      Holds the application's state and sensor data.
 * - Sensors:        Manages reading from the temperature and current/voltage sensors.
 * - Power:          Controls the charging PWM output and the current model.
 * - InternalResistance: Manages the internal resistance measurement process.
 * - DisplayManager: Handles all drawing and updates to the TFT screen.
 * - Remote:         Processes commands from the IR remote control.
 *
 * A FreeRTOS task is used for non-blocking sensor reads.
 * --------------------------------------------------------------------
 */

#include "config.h"
#include "Shared.h"
#include "DataStore.h"
#include "Sensors.h"
#include "Power.h"
#include "InternalResistance.h"
#include "DisplayManager.h"
#include "Remote.h"

// --- Global Objects ---
// These objects instantiate the main components of the application.
// They are passed pointers to each other as needed to facilitate
// communication between modules.
DataStore dataStore;
Sensors sensors(&dataStore);
Power power(&dataStore);
InternalResistance ir_tester(&dataStore, &power);
DisplayManager displayManager(&dataStore);
Remote remote(&dataStore, &power, &ir_tester, &displayManager);


// --- FreeRTOS Task for Sensor Reading ---
// This task runs in the background, continuously reading from the
// sensors and updating the central DataStore. This decouples data
// acquisition from the main application logic.
void task_readSensors(void* parameter) {
    while (true) {
        sensors.read();
        // The delay inside sensors.read() is sufficient,
        // but a small vTaskDelay can prevent watchdog issues if that loop changes.
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- Arduino Setup Function ---
// This function runs once at startup. It initializes hardware,
// starts the IR receiver, sets up the FreeRTOS task, and prints
// initial instructions to the Serial monitor.
void setup() {
    Serial.begin(115200);

    // Initialize the TFT display
    displayManager.begin();

    // Start the IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN);

    // Initialize sensor hardware
    sensors.begin();

    // Initialize power management (PWM output)
    power.begin();

    // Create the background task for reading sensors
    xTaskCreate(task_readSensors, "SensorRead", 4096, NULL, 1, NULL);

    // Print welcome message and instructions
    Serial.println("-------------------------------------------------");
    Serial.println("Ni-Cd/Ni-MH Battery Charger - Refactored");
    Serial.println("Controlled by a Samsung DVD remote.");
    Serial.println("-------------------------------------------------");
    Serial.println("- POWER:  Start charging cycle (builds model first)");
    Serial.println("- PLAY:   Measure internal resistance of battery");
    Serial.println("- INFO:   Show internal resistance graph");
    Serial.println("- SOURCE: Show the main charging graph");
    Serial.println("-------------------------------------------------");
}

// --- Arduino Loop Function ---
// This is the main event loop of the application. It continuously
// calls the update() methods of the various modules. These methods
// contain the state machines and logic for handling timed events.
// It also polls for IR remote commands.
void loop() {
    // Update the state machines for the core application logic
    power.update();
    ir_tester.update();

    // Update the display based on the current state
    displayManager.update();

    // Poll for and handle incoming IR commands
    remote.handle();

    // A small delay to be a good citizen, though the display update
    // interval largely controls the loop rate.
    delay(10);
}