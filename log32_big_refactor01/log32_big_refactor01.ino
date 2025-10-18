#include "Shared.h"
#include "DataStore.h"
#include "DisplayManager.h"
#include "Sensors.h"
#include "Power.h"
#include "Remote.h"
#include "internal_resistance.h"

// --- Global Objects ---
DataStore data;
DisplayManager displayManager;
Sensors sensors(data);
Power power;
Remote remote(data);

// --- Global Variable Definitions ---
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile float mAh_charged = 0.0f;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;

unsigned long lastPlotUpdateTime = 0;
unsigned long lastChargingHouseTime = 0;
unsigned long lastIRHandleTime = 0;
unsigned long displayStateChangeTime = 0;

// --- Task Definitions ---
void task_readSensors(void* parameter) {
    while (true) {
        sensors.read();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    displayManager.begin();
    sensors.begin();
    power.begin();
    remote.begin();

    xTaskCreate(task_readSensors, "Sensors", 4096, NULL, 1, NULL);

    Serial.println("Ni-Cd/Ni-MH battery charger. use samsung DVD remote to control");
    Serial.println("PLAY to measure internal resistance of battery");
    Serial.println("INFO to see internal resistance graph");
    Serial.println("POWER to start charging");
    Serial.println("SOURCE to see charge graph");
}

void loop() {
    unsigned long now = millis();

    // --- State Machine ---
    switch (data.currentAppState) {
        case APP_STATE_IDLE:
            // Nothing to do
            break;
        case APP_STATE_BUILDING_MODEL:
            power.buildCurrentModel(data);
            break;
        case APP_STATE_MEASURING_IR:
            measureInternalResistanceStep(data);
            break;
        case APP_STATE_CHARGING:
            if (now - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = now;
                if (!power.chargeBattery(data)) {
                    data.currentAppState = APP_STATE_IDLE;
                }
            }
            break;
    }

    // --- Timed Events ---
    if (now - lastPlotUpdateTime >= PLOT_UPDATE_INTERVAL_MS) {
        lastPlotUpdateTime = now;
        displayManager.update(data);
    }

    if (data.currentDisplayState != DISPLAY_STATE_MAIN && now - displayStateChangeTime > 20000) {
        data.currentDisplayState = DISPLAY_STATE_MAIN;
    }

    // --- IR Remote Handling ---
    if (now - lastIRHandleTime >= IR_HANDLE_INTERVAL_MS) {
        lastIRHandleTime = now;
        remote.handle();
    }
}