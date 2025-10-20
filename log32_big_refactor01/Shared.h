#ifndef SHARED_H
#define SHARED_H

#include <cstdint>

// This file will be populated with shared enums and data structures.

enum AppState {
    APP_STATE_IDLE,
    APP_STATE_BUILDING_MODEL,
    APP_STATE_MEASURING_IR,
    APP_STATE_CHARGING
};

// Represents the state of the internal resistance measurement process
enum class IRState {
    IDLE,
    START,
    FIND_MIN_CURRENT,
    MEASURE_PAIRS,
    CALCULATE,
    DISPLAY_RESULTS, // Renamed from DISPLAY to avoid macro conflict
    COMPLETE,
    ABORTED
};

struct MeasurementData {
    float voltage;
    float current;
    double temp1;
    double temp2;
    double tempDiff;
    float t1_millivolts;
    int dutyCycle;
    unsigned long timestamp;
};

enum class LabelVerticalPlacement {
    CENTER,
    ABOVE,
    BELOW
};

struct Label {
    std::string text;
    int x;
    int y;
    uint16_t color;
    int textWidth;
    int textHeight;
    int lineStartX;
    int lineEndX;
    int y_initial;
    int lineY;
    LabelVerticalPlacement verticalPlacement;
    float minValue;
    float maxValue;
};

struct CurrentModel {
    Eigen::VectorXd coefficients;
    bool isModelBuilt = false;
};

struct TickLabel {
    int y;
    float value;
    uint16_t color;
    Label label;
};

struct ChargeLogData {
    unsigned long timestamp;
    float current;
    float voltage;
    float ambientTemperature;
    float batteryTemperature;
    int dutyCycle;
    float internalResistanceLoadedUnloaded;
    float internalResistancePairs;
};

struct MHElectrodeData {
    float unloadedVoltage;
    float loadedVoltage;
    float targetVoltage;
    float voltageDifference;
    float current;
    uint32_t dutyCycle;
    uint32_t timestamp;
};

enum ChargingState {
    CHARGE_IDLE = 0,
    CHARGE_FIND_OPT,
    CHARGE_MONITOR,
    CHARGE_STOPPED
};

enum FindPhase {
    FIND_IDLE,
    FIND_INIT_HIGHDC,
    FIND_BINARY_PREPARE,
    FIND_BINARY_WAIT,
    FIND_FINAL_WAIT,
    FIND_COMPLETE
};

struct FindOptManager {
    bool active = false;
    int maxDC = 0;
    int lowDC = 0;
    int highDC = 0;
    int optimalDC; // MIN_CHARGE_DUTY_CYCLE
    float closestVoltageDifference = 1000.0f;
    float targetVoltage = 0.0f;
    float initialUnloadedVoltage = 0.0f;
    std::vector<MHElectrodeData> cache;
    FindPhase phase = FIND_IDLE;
    bool isReevaluation = false;
};

enum MeasState {
    MEAS_IDLE,
    MEAS_STOPLOAD_WAIT,
    MEAS_SAMPLE_UNLOADED,
    MEAS_APPLY_LOAD,
    MEAS_SAMPLE_LOADED,
    MEAS_COMPLETE,
    MEAS_ABORTED
};

struct AsyncMeasure {
    MeasState state = MEAS_IDLE;
    int testDuty = 0;
    unsigned long stateStart = 0;
    unsigned long unloadedDelay = 0;
    unsigned long stabilizationDelay = 0;
    MeasurementData unloadedData;
    MeasurementData loadedData;
    MHElectrodeData result;
    bool resultReady = false;
    bool active() const { return state != MEAS_IDLE && state != MEAS_COMPLETE && state != MEAS_ABORTED; }
    void reset() {
        state = MEAS_IDLE;
        testDuty = 0;
        stateStart = 0;
        unloadedDelay = 0;
        stabilizationDelay = 0;
        resultReady = false;
    }
};

namespace RemoteKeys {
  enum KeyCode {
    KEY_POWER = 0xE6,
    KEY_0 = 0x11, KEY_1 = 0x04, KEY_2 = 0x05, KEY_3 = 0x06, KEY_4 = 0x08,
    KEY_5 = 0x09, KEY_6 = 0x0A, KEY_7 = 0x0C, KEY_8 = 0x0D, KEY_9 = 0x0E,
    KEY_UP = 0x60, KEY_DOWN = 0x61, KEY_LEFT = 0x65, KEY_RIGHT = 0x62,
    KEY_OK = 0x68, KEY_MENU = 0x79, KEY_RED = 0x6c, KEY_GREEN = 0x14,
    KEY_YELLOW = 0x15, KEY_BLUE = 0x16, KEY_VOL_UP = 0x07, KEY_VOL_DOWN = 0x0b,
    KEY_CH_UP = 0x12, KEY_CH_DOWN = 0x10, KEY_REWIND = 0x45, KEY_PLAY = 0x47,
    KEY_PAUSE = 0x4A, KEY_FORWARD = 0x48, KEY_STOP = 0x46, KEY_SETTINGS = 0x1A,
    KEY_INFO = 0x1F, KEY_SUBTITLES = 0x25, KEY_MUTE = 0x0F, KEY_NETFLIX = 0xF3,
    KEY_PRIME_VIDEO = 0xF4, KEY_GUIDE = 0x4F, KEY_SOURCE = 0x01
  };
}

#endif // SHARED_H