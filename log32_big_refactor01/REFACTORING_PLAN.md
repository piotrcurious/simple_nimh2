# Refactoring Plan for log32_big_refactor01

This document outlines the plan for refactoring the Arduino project into a modular, maintainable, and understandable structure.

## 1. Core Principles

- **Separation of Concerns:** Each module has a single, well-defined responsibility.
- **Centralized Data Management:** A single `DataStore` module will be the source of truth for all application state.
- **Unidirectional Data Flow:** Data should flow in a single direction: `Sensors` -> `DataStore` -> `Power`/`DisplayManager`.
- **Incremental Changes:** The refactoring will be done in small, verifiable steps.

## 2. Target Architecture

### Modules

1.  **`config.h`**: Holds all hardware-specific definitions (pins, ADC settings, etc.).
2.  **`Shared.h`**: Defines shared, simple data structures and enumerations (`AppState`, `DisplayState`, `MeasurementData`, etc.).
3.  **`DataStore.h`/`.cpp`**: A central class (`DataStore`) that encapsulates all runtime application state. This includes history arrays for plotting, the charge log, resistance measurement data, and application state variables.
4.  **`Sensors.h`/`.cpp`**: A `Sensors` class responsible for initializing and reading from all hardware sensors. It will run in a FreeRTOS task and continuously update the `DataStore`.
5.  **`DisplayManager.h`/`.cpp`**: A `DisplayManager` class that reads from the `DataStore` and orchestrates which plotter to use based on the current display state.
6.  **Plotter Classes** (`MainPlotter.h`/`.cpp`, `ChargePlotter.h`/`.cpp`, `ResistancePlotter.h`/`.cpp`): Each class is responsible for drawing a single screen or graph. They are given data by the `DisplayManager`.
7.  **`Power.h`/`.cpp`**: A `Power` class that manages the charging state machine and PWM output. It reads from the `DataStore` and can request actions like resistance measurements.
8.  **`Remote.h`/`.cpp`**: A `Remote` class to handle IR input, which updates state in the `DataStore`.
9.  **`internal_resistance.h`/`.cpp`**: A module containing the logic for the internal resistance measurement state machine.
10. **`log32_big_refactor01.ino`**: The main application file, responsible for initialization, task creation, and the main `loop` which acts as a state machine coordinator.

### Data and Control Flow

1.  **Initialization (`setup()`)**:
    - The main `.ino` file will instantiate all module classes (`DataStore`, `Sensors`, `Power`, `DisplayManager`, `Remote`).
    - It will initialize each module.
    - It will create the FreeRTOS tasks for sensor reading.
    - It will set up the `Remote` class with lambdas to modify the `AppState` in the `DataStore`.
2.  **Main Loop (`loop()`)**:
    - The `loop` will be very simple.
    - It will read the current `AppState` from the `DataStore`.
    - Based on the state, it will call the appropriate module's `update()` or `step()` method (e.g., `power.update(data)`).
    - It will periodically call `displayManager.update(data)` to refresh the screen.
    - It will periodically call `remote.handle()` to check for IR input.
3.  **Sensor Data**:
    - The `Sensors` object, running in a FreeRTOS task, will continuously read from the hardware sensors.
    - It will write the latest sensor readings and history data into the `DataStore` object. This is the *only* module that writes sensor data.
4.  **Display Data**:
    - The `DisplayManager` will read all necessary data from the `DataStore` object.
    - It will then pass the relevant subsets of that data to the active plotter class. The plotters themselves will be stateless.

## 3. Micro-Plans for Large Functions

This section identifies functions that are too large to be refactored in a single step and provides a micro-plan for each.

### 3.1. `buildCurrentModelStep()`

*   **Location:** `log32_big_refactor01.ino` -> `Power.cpp`
*   **Challenge:** This function is a state machine that also has direct dependencies on sensor readings and display updates.
*   **Micro-Plan:**
    1.  Move the state machine variables (`buildModelStep`, `buildModelDutyCycle`, etc.) into the `Power` class as private members.
    2.  Move the entire function into `Power::buildCurrentModelStep()`.
    3.  Refactor the direct call to `getThermistorReadings` to accept a `MeasurementData` object as a parameter.
    4.  Refactor the direct call to `processThermistorData` to be a function callback parameter.
    5.  In the main loop, when the state is `APP_STATE_BUILDING_MODEL`, call `power.buildCurrentModelStep()`, passing in lambdas that get data from the `DataStore` and update the display via the `DisplayManager`.

### 3.2. `chargeBattery()`

*   **Location:** `charging.cpp` -> `Power.cpp`
*   **Challenge:** This is a complex state machine with dependencies on sensor data, internal resistance measurements, and display updates.
*   **Micro-Plan:**
    1.  Move the state variables (`chargingState`, `lastChargeEvaluationTime`, etc.) into the `Power` class as private members.
    2.  Move the entire `chargeBattery` function into `Power.cpp`.
    3.  Refactor the direct calls to `getThermistorReadings` to use data passed in from the `DataStore`.
    4.  Refactor the calls to `startFindOptimalManagerAsync` and `findOptimalChargingDutyCycleStepAsync` to interact with the `internal_resistance` module.
    5.  Refactor the logging calls (`logChargeData`, `pushRecentChargeLog`) to call methods on the `DataStore` object.

### 3.3. `displayInternalResistanceGraph()` and `drawChargePlot()`

*   **Location:** `graphing.cpp` -> `ResistancePlotter.cpp` and `ChargePlotter.cpp`
*   **Challenge:** These functions contain complex drawing logic and direct access to global data arrays.
*   **Micro-Plan:**
    1.  Move the core drawing logic for each function into its respective plotter class (`ResistancePlotter::draw`, `ChargePlotter::draw`).
    2.  Change the function signatures to accept all necessary data (e.g., `const std::vector<ChargeLogData>& chargeLog`) as a `const` reference parameter. This makes the plotters stateless.
    3.  Move any helper functions (e.g., `extractValidData`, `findMinMax`) into the anonymous namespace of the respective `.cpp` file to limit their scope.
    4.  In the `DisplayManager::update` method, read the necessary data from the `DataStore` and pass it to the `draw` method of the active plotter.

By following this detailed plan, I will address the architectural issues and my own execution limitations, leading to a successful and robust refactoring.