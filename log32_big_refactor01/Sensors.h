#ifndef SENSORS_H
#define SENSORS_H

#include "DataStore.h"
#include "SHT4xSensor.h"
#include "ThermistorSensor.h"
#include "analog.h" // Use the existing analog reading functions

// The Sensors class is responsible for managing all hardware sensors.
// It initializes them and provides a single method (`read()`) to
// acquire fresh data from all sources and update the central DataStore.
class Sensors {
public:
    // --- Constructor ---
    // Takes a pointer to the central DataStore to update it directly.
    Sensors(DataStore* data_store);

    // --- Public Methods ---
    void begin(); // Initializes the sensor hardware.
    void read();  // Reads from all sensors and updates the DataStore.

private:
    // --- Private Members ---
    DataStore* _data_store; // Pointer to the central data repository.
    SHT4xSensor _sht4x_sensor;
    ThermistorSensor _thermistor_sensor;

    // Timing for non-blocking reads
    uint32_t _last_voltage_update_time;
    uint32_t _last_mah_update_time;

    // --- Private Helper Methods ---
    void _read_and_update_all_sensors();
    void _update_mah_charged();
};

#endif // SENSORS_H