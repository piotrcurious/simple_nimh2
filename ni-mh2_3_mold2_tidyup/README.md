# Ni-MH/Ni-Cd Smart Battery Charger

This project is an advanced battery charger for Ni-MH and Ni-Cd cells, built on the ESP32 platform. It goes beyond simple charging by incorporating detailed real-time monitoring, internal resistance measurement, and a graphical user interface to visualize the battery's health and charging progress. The system is controlled wirelessly using a standard IR remote.

## Hardware Setup

The hardware is designed to precisely control the charging process while monitoring key battery parameters.

### Core Components
- **Microcontroller:** ESP32
- **Display:** TFT LCD Screen for displaying graphs and data.
- **Input:** IR Receiver for remote control.

### Charging Circuit
The charging current is delivered to the battery using a **NPN Darlington transistor** controlled by a PWM signal from the ESP32. This allows for fine-grained control over the charging current.
- The battery's positive terminal is connected directly to the main power supply (VCC).
- The battery's negative terminal is connected to the collector of the NPN transistor.

### Current Measurement
Current is measured using a **current shunt resistor** placed between the transistor's emitter and ground.
- A large **2200uF capacitor** is placed in parallel with the shunt resistor. This smooths the pulsed current from the PWM signal, allowing the analog-to-digital converter (ADC) to measure the average (DC) current flowing into the battery.
- The code attempts to factor this hardware setup into its current calculations, though it is noted that the implementation may not be perfectly accurate.

### Temperature Sensing
Temperature is monitored using a **thermistor voltage divider**:
- One thermistor is connected between VCC and the analog input pin (top thermistor).
- A second thermistor is connected between the analog input pin and ground (bottom thermistor).
- This configuration allows for differential temperature measurements, which can be used to detect the end of a charge cycle (using the -ΔV/Δt method) or identify fault conditions like overheating.

### Voltage Reference
To ensure stable and accurate analog readings, the system uses a **ratiometric reference voltage**:
- A trimpot is configured as a voltage divider to supply exactly half of VCC to a dedicated analog input.
- The firmware uses this reference to correct for any fluctuations in the main VCC supply, leading to more reliable voltage and temperature measurements.

## Code Structure

The project is organized into several modules, each responsible for a specific aspect of the charger's functionality. This modular design improves code readability and maintainability.

- **`ni-mh2_3_mold2_tidyup.ino`**: The main application file. It contains the `setup()` and `loop()` functions, which initialize the system and manage the main application state machine. It ties all the other modules together.

- **`definitions.h`**: A central header file that defines all global constants, pin assignments, data structures (`struct`s), and enumerations (`enum`s) used across the project. It also contains function prototypes for functions used by multiple modules.

- **`charging.h` / `charging.cpp`**: This module manages the battery charging logic. It includes functions to start and stop charging, as well as the state machine for the charging process itself. It also contains the logic for estimating the battery's temperature rise during charging.

- **`graphing.h` / `graphing.cpp`**: This module is responsible for all display-related functions. It handles drawing the real-time graphs of temperature, voltage, and current on the TFT screen. It also manages the display of labels and other UI elements.

- **`internal_resistance.h` / `internal_resistance.cpp`**: This module implements the functionality to measure the battery's internal resistance. This is a key diagnostic feature for assessing the health of the battery. It includes a state machine to manage the multi-step measurement process.

- **`analog.h` / `analog.cpp`**: Provides low-level functions for reading analog values from the various sensors (thermistors, voltage dividers, current shunt). It likely includes oversampling and other techniques to improve measurement accuracy.

- **`SHT4xSensor.h` / `SHT4xSensor.cpp`**: A sensor driver for the SHT4x temperature and humidity sensor. This is likely used for measuring the ambient temperature.

- **`ThermistorSensor.h` / `ThermistorSensor.cpp`**: This module encapsulates the logic for reading and converting the raw ADC values from the thermistor voltage divider into temperature readings.

- **`home_screen.h` / `home_screen.cpp`**: Manages the content and rendering of the idle or home screen of the user interface.

- **`logging.h` / `logging.cpp`**: Implements data logging functionality, likely for storing charge cycle data for later analysis.

## Key Software Concepts

The firmware employs several key software design patterns to manage the complexity of the charging and monitoring process.

### State Machines
The application's logic is built around two primary state machines:
- **`AppState`**: Manages the overall state of the charger (e.g., `APP_STATE_IDLE`, `APP_STATE_CHARGING`, `APP_STATE_MEASURING_IR`).
- **`DisplayState`**: Controls what is currently being shown on the TFT display (e.g., `DISPLAY_STATE_IDLE`, `DISPLAY_STATE_MAIN`, `DISPLAY_STATE_IR_GRAPH`).
This separation of concerns allows the application to perform background tasks (like charging) while the user navigates through different display screens.

### Real-Time Tasks (FreeRTOS)
The ESP32's dual-core architecture is leveraged using FreeRTOS tasks to handle sensor readings concurrently and without blocking the main application loop.
- **`task_readSHT4x`**: Periodically reads data from the SHT4x ambient temperature/humidity sensor.
- **`task_readThermistor`**: Periodically reads the battery temperature, voltage, and current. This task also calculates the total charge (mAh) delivered to the battery.

### Current Estimation Model
Due to the non-linear relationship between PWM duty cycle and charging current, the firmware builds a mathematical model to accurately predict the current for a given duty cycle.
- The `buildCurrentModelStep()` function automatically sweeps through a range of PWM duty cycles and measures the resulting current.
- It then uses a polynomial regression (via the `Eigen` library) to fit a curve to this data.
- This model is then used to estimate the current when it is too low to be measured accurately, and to help determine the optimal charging current.

### Remote Control Interface
The charger is controlled using a Samsung DVD remote. The `handleIRCommand()` function maps specific remote control keys to actions within the application:
- **PLAY**: Starts the internal resistance measurement process.
- **INFO**: Displays the internal resistance graph.
- **POWER**: Starts the charging process (which begins by building the current model).
- **SOURCE**: Shows the main charging graph.
- **MENU**: Toggles between the main display and the idle/home screen.
