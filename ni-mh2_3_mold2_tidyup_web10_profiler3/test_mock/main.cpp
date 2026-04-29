#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <stdarg.h>
#include <assert.h>
#include <algorithm>

#include "dummy_esp32.h"

// Math helpers
using std::isnan;
using std::isfinite;
using std::min;
using std::max;

MockSerial Serial;
unsigned long mock_millis = 0;
uint64_t mock_esp_timer_now = 0;

// Redirect definitions.h includes
#define SPI_h
#define Arduino_h
#define WiFi_h
#define Adafruit_SHT4x_h
#define ADC_CONTINUOUS_H
#define ESP_ADC_CAL_H
#define ADC_H
#define WEBSERVER_H

#include "../adc_dma.h"

// We need to include SystemDataManager.h with private members accessible
#define private public
#include "../SystemDataManager.h"
#undef private

#include "../definitions.h"
#include "../internal_resistance.h"
#include "../AdvancedPolynomialFitter.hpp"

extern AsyncWebSocket ws;

// Profile mocks
CoreBuf g_coreBuf[CORE_COUNT];
volatile uint32_t g_frameSeq = 0;
volatile uint32_t g_frameStartUs = 0;
void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags) {
    if (core < CORE_COUNT && g_coreBuf[core].count < MAX_EVENTS_PER_CORE) {
        auto& e = g_coreBuf[core].events[g_coreBuf[core].count++];
        e.taskId = taskId; e.flags = flags; e.startUs = startUs; e.durUs = durUs;
    }
}
void sendFramePacket(bool timeoutFlag) {
    uint8_t buf[1024];
    buf[0] = 0x50; buf[1] = 0x54; // TP LE
    buf[14] = g_coreBuf[0].count;
    buf[15] = g_coreBuf[1].count;
    ws.binaryAll(buf, 18 + (g_coreBuf[0].count + g_coreBuf[1].count) * 6);
}

WebServer mock_server;
AsyncWebSocket ws("/ws");

const double BCOEFFICIENT = 3950.0;

// Hardware stubs that would normally be in .ino
void applyDuty(uint32_t duty);
void setAppState(AppState s);
void setBuildModelPhase(BuildModelPhase p);
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current);
float estimateCurrent(int duty);
int estimateDutyCycleForCurrent(float target);
void handleData(AsyncWebServerRequest *request);
void handleCommand(AsyncWebServerRequest *request);
void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot);
uint32_t calculateSnapshotAverage(const AdcSnapshot &old_s, const AdcSnapshot &new_s);
float snapshotToMillivolts(AdcChannelIndex idx, uint32_t avg_raw);
void setupAdcDma() {}
void processAdcDma() {}
double calculateBatteryTemp(double ambientTemp, float therm1Mv, float vccMv);

// Physics Simulation
struct BatterySim {
    float voltage = 1.15f;
    float unloaded_voltage = 1.25f;
    float temp = 22.0f;
    float ambient = 22.0f;
    float soc = 0.05f;
    float internal_resistance = 0.15f;
    float capacity_ah = 2.0f;

    // Non-linear duty-to-current mapping
    float getCurrent(int duty) {
        if (duty < 20) return 0.0f;
        float normalized = (float)(duty - 20) / 235.0f;
        return 2.5f * (normalized * normalized);
    }

    void update(float dt_s, int duty) {
        float current = getCurrent(duty);
        soc += (current * dt_s) / (capacity_ah * 3600.0f);

        float base_v = 1.2f + 0.15f * std::pow(soc, 0.5f);
        if (soc > 0.9f) base_v += (soc - 0.9f) * 4.0f;

        float current_ir = internal_resistance * (1.0f + std::max(0.0f, (float)std::abs(0.5f - soc) * 0.5f));
        unloaded_voltage = base_v;
        voltage = base_v + current * current_ir;

        float efficiency = 1.0f;
        if (soc > 0.8f) efficiency = 1.0f - (soc - 0.8f) * 2.0f;
        if (efficiency < 0) efficiency = 0;
        float P_heat = current * current * current_ir + current * voltage * (1.0f - efficiency);

        float dT = (P_heat * dt_s) / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT);
        float cooling = (temp - ambient) * 0.05f * dt_s;
        temp += dT - cooling;
    }
};
BatterySim sim;

// Global variables from .ino
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile float MEASURABLE_CURRENT_THRESHOLD = 0.005f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;
uint32_t dutyCycle = 0;
bool isCharging = false;
volatile AppState currentAppState = APP_STATE_IDLE;
volatile AppState postModelAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;
unsigned long lastPlotUpdateTime = 0;
unsigned long lastChargingHouseTime = 0;
const int pwmPin = 19;
double THERMISTOR_1_OFFSET = 0.0;

void applyDuty(uint32_t duty) { dutyCycle = duty; }

void AsyncWebServerRequest::send(int code, const char* type, String content) {
    mock_server.lastResponseCode = code;
    mock_server.lastResponseType = type;
    mock_server.lastResponseContent = content;
}

void AsyncWebServerRequest::send(AsyncWebServerResponse* response) {
    if (response->_isChunked && response->_callback) {
        mock_server.lastResponseContent = "";
        uint8_t buf[1024];
        size_t index = 0;
        size_t written;
        while ((written = response->_callback(buf, sizeof(buf), index)) > 0) {
            mock_server.lastResponseContent.append((const char*)buf, written);
            index += written;
        }
    } else {
        mock_server.lastResponseContent = response->_content;
    }
    delete response;
}

void setAppState(AppState s) { currentAppState = s; }
void setBuildModelPhase(BuildModelPhase p) { buildModelPhase = p; }

void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    SystemData d = systemData.getData();
    temp1 = d.ambient_temp_c;
    temp2 = d.battery_temp_c;
    tempDiff = d.temp_diff_c;
    t1_millivolts = 0;
    voltage = d.battery_voltage_v;
    current = d.charge_current_a;
}

// ADC DMA Stubs for SystemDataManager.cpp
static uint32_t mock_sample_count = 0;
void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot) {
    snapshot.count = mock_sample_count;
    float val_mv = 0;
    if (idx == ADC_IDX_CURRENT) {
        val_mv = sim.getCurrent((int)dutyCycle) * CURRENT_SHUNT_RESISTANCE * 1000.0f + systemData.getCurrentZeroOffsetMv();
    } else if (idx == ADC_IDX_VOLTAGE) {
        // Correct the voltage mapping to match SystemDataManager logic: batteryV = ((vcc_mv * MAIN_VCC_RATIO) - sampledMv) / 1000.0f
        // So sampledMv = (vcc_mv * MAIN_VCC_RATIO) - (batteryV * 1000)
        float batteryV = (dutyCycle == 0 ? sim.unloaded_voltage : sim.voltage);
        val_mv = (3300.0f * MAIN_VCC_RATIO) - (batteryV * 1000.0f);
    } else if (idx == ADC_IDX_THERM1) {
        // Correct thermistor mapping to match calculateBatteryTemp logic
        // vRatio = averageAnalogValue / ((vcc_millivolts * MAIN_VCC_RATIO) - averageAnalogValue)
        // We want battTemp = sim.temp
        double battK = sim.temp + 273.15;
        double ambK = sim.ambient + 273.15;
        double logVRatio = ( (1.0/battK) - (1.0/ambK) ) * BCOEFFICIENT;
        double vRatio = exp(logVRatio);
        // vRatio = val_mv / ( (3300 * MAIN_VCC_RATIO) - val_mv )
        // vRatio * (3300*R - val_mv) = val_mv
        // vRatio * 3300 * R = val_mv * (1 + vRatio)
        val_mv = (vRatio * 3300.0f * MAIN_VCC_RATIO) / (1.0 + vRatio);
    } else if (idx == ADC_IDX_THERM_VCC) {
        val_mv = 3300.0f;
    }
    snapshot.sum = (uint64_t)val_mv * snapshot.count;
}
uint32_t calculateSnapshotAverage(const AdcSnapshot &old_s, const AdcSnapshot &new_s) {
    uint32_t d_count = new_s.count - old_s.count;
    if (d_count == 0) return (uint32_t)(new_s.sum / (new_s.count ? new_s.count : 1));
    return (uint32_t)((new_s.sum - old_s.sum) / d_count);
}
float snapshotToMillivolts(AdcChannelIndex idx, uint32_t avg_raw) {
    return (float)avg_raw;
}

double calculateBatteryTemp(double ambientTemp, float therm1Mv, float vccMv) {
    double averageAnalogValue = (double)therm1Mv - 0; // therm1Offset is 0 in mock
    double vcc_millivolts = (double)vccMv;

    double vRatio = averageAnalogValue / ((vcc_millivolts * MAIN_VCC_RATIO) - averageAnalogValue);
    if (vRatio <= 0) return ambientTemp;

    double logVRatio = log(vRatio);
    double ambientKelvin = ambientTemp + 273.15;

    double invBattKelvin = (1.0 / ambientKelvin) + (logVRatio / BCOEFFICIENT);
    double battKelvin = 1.0 / invBattKelvin;
    return battKelvin - 273.15;
}



SHT4xSensor sht4Sensor;
SystemDataManager systemData(sht4Sensor, 36, 35, 0.0);
CurrentModel currentModel;

#define private public
#include "../home_screen.h"
#undef private
HomeScreen homeScreen;

WebSocketsServer webSocket(81);

#include "../charging.h"
#include "../logging.h"
#include "../graphing.h"

// Manually bring in parts of .ino for testing model build
extern void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
extern void broadcastLiveTelemetry();
volatile BuildModelPhase buildModelPhase = BuildModelPhase::Idle;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
float mock_calibrationSum = 0;
float mock_calibrationMax = 0;
int mock_calibrationCount = 0;
uint32_t mock_lastKnownSampleCount = 0;
float mock_noiseFloorMv = 0;
volatile float noiseFloorMv = 0;
std::vector<float> mock_dutyCycles;
std::vector<float> mock_currents;

void buildCurrentModelStep() {
    const unsigned long now = millis();
    switch (buildModelPhase) {
        case BuildModelPhase::Idle:
            mock_dutyCycles.clear(); mock_currents.clear();
            applyDuty(0);
            buildModelLastStepTime = now;
            buildModelPhase = BuildModelPhase::Settle;
            std::cout << "  Phase Idle -> Settle at " << now << std::endl;
            break;
        case BuildModelPhase::Settle:
            if (now - buildModelLastStepTime >= 2000) {
                mock_calibrationSum = 0; mock_calibrationMax = 0; mock_calibrationCount = 0;
                mock_lastKnownSampleCount = systemData.getData().current_sample_count;
                buildModelLastStepTime = now;
                buildModelPhase = BuildModelPhase::Calibrate;
                std::cout << "  Phase Settle -> Calibrate at " << now << std::endl;
            }
            break;
        case BuildModelPhase::Calibrate:
            {
                SystemData d = systemData.getData();
                if (d.current_sample_count != mock_lastKnownSampleCount) {
                    mock_lastKnownSampleCount = d.current_sample_count;
                    mock_calibrationSum += d.current_mv;
                    if (d.current_mv > mock_calibrationMax) mock_calibrationMax = d.current_mv;
                    mock_calibrationCount++;
                }
                if (mock_calibrationCount >= 20) {
                    float avg = mock_calibrationSum / (float)mock_calibrationCount;
                    systemData.setCurrentZeroOffsetMv(avg);
                    mock_noiseFloorMv = (mock_calibrationMax - avg) * 2.0f;
                    if (mock_noiseFloorMv < 2.5f) mock_noiseFloorMv = 2.5f;
                    buildModelDutyCycle = 1;
                    applyDuty(buildModelDutyCycle);
                    buildModelLastStepTime = now;
                    buildModelPhase = BuildModelPhase::DetectDeadRegion;
                    std::cout << "  Phase Calibrate -> DetectDeadRegion at " << now << " (Offset " << avg << ", Noise " << mock_noiseFloorMv << ")" << std::endl;
                }
            }
            break;
        case BuildModelPhase::DetectDeadRegion:
            if (now - buildModelLastStepTime >= 250) {
                SystemData d = systemData.getData();
                float currentMv = d.current_mv - systemData.getCurrentZeroOffsetMv();
                if (currentMv > mock_noiseFloorMv) {
                    MEASURABLE_CURRENT_THRESHOLD = d.charge_current_a;
                    mock_dutyCycles.push_back(0.0f); mock_currents.push_back(0.0f);
                    mock_dutyCycles.push_back((float)buildModelDutyCycle); mock_currents.push_back(d.charge_current_a);
                    buildModelDutyCycle += 5;
                    applyDuty(buildModelDutyCycle);
                    buildModelLastStepTime = now;
                    buildModelPhase = BuildModelPhase::WaitMeasurement;
                    std::cout << "  Phase DetectDeadRegion -> WaitMeasurement at " << now << " (Duty " << buildModelDutyCycle << ", Current " << d.charge_current_a << ")" << std::endl;
                } else {
                    buildModelDutyCycle += 2;
                    if (buildModelDutyCycle > MAX_DUTY_CYCLE) {
                        std::cout << "  Phase DetectDeadRegion -> ABORTED (Max Duty reached)" << std::endl;
                        currentAppState = APP_STATE_IDLE;
                        buildModelPhase = BuildModelPhase::Idle;
                    } else {
                        applyDuty(buildModelDutyCycle);
                    }
                }
                buildModelLastStepTime = now;
            }
            break;
        case BuildModelPhase::SetDuty:
            if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
                applyDuty(buildModelDutyCycle);
                buildModelLastStepTime = now;
                buildModelPhase = BuildModelPhase::WaitMeasurement;
            } else buildModelPhase = BuildModelPhase::Finish;
            break;
        case BuildModelPhase::WaitMeasurement:
            if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
                SystemData d = systemData.getData();
                if (d.charge_current_a >= MEASURABLE_CURRENT_THRESHOLD) {
                    mock_dutyCycles.push_back((float)buildModelDutyCycle);
                    mock_currents.push_back(d.charge_current_a);
                }
                buildModelDutyCycle += 5;
                if (buildModelDutyCycle <= MAX_DUTY_CYCLE) {
                    applyDuty(buildModelDutyCycle);
                    buildModelLastStepTime = now;
                } else {
                    buildModelPhase = BuildModelPhase::Finish;
                }
            }
            break;
        case BuildModelPhase::Finish:
            if (mock_dutyCycles.size() >= 2) {
                int degree = 3;
                AdvancedPolynomialFitter fitter;
                std::vector<float> coeffs = fitter.fitPolynomialLebesgue(mock_dutyCycles, mock_currents, degree);

                currentModel.coefficients.resize(coeffs.size());
                for (size_t i = 0; i < (size_t)coeffs.size(); ++i) {
                    currentModel.coefficients(i) = coeffs[i];
                }

                if (degree >= 0) currentModel.coefficients(0) = 0.0;
                currentModel.isModelBuilt = true;
                applyDuty(0);
                if (postModelAppState == APP_STATE_CHARGING) {
                    currentAppState = APP_STATE_CHARGING;
                    startCharging();
                } else if (postModelAppState == APP_STATE_MEASURING_IR) {
                    currentIRState = IR_STATE_START;
                    currentAppState = APP_STATE_MEASURING_IR;
                } else {
                    currentAppState = APP_STATE_IDLE;
                }
            } else {
                std::cout << "  Phase Finish -> ABORTED (Not enough points: " << mock_dutyCycles.size() << ")" << std::endl;
                currentAppState = APP_STATE_IDLE;
            }
            postModelAppState = APP_STATE_IDLE;
            buildModelPhase = BuildModelPhase::Idle;
            std::cout << "  Phase Finish at " << now << " (Built: " << currentModel.isModelBuilt << ", Points: " << mock_dutyCycles.size() << ")" << std::endl;
            break;
    }
}

float estimateCurrent(int duty) {
    if (!currentModel.isModelBuilt) return sim.getCurrent(duty);
    double sum = 0.0;
    for (int i = 0; i < currentModel.coefficients.size(); ++i) sum += currentModel.coefficients(i) * std::pow((float)duty, i);
    return (float)std::max(0.0, sum);
}

int estimateDutyCycleForCurrent(float targetCurrent) {
    int bestDC = 0;
    float closestCurrentDiff = std::numeric_limits<float>::max();
    for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; ++dc) {
        float estimated = estimateCurrent(dc);
        float diff = std::abs(estimated - targetCurrent);
        if (diff < closestCurrentDiff) { closestCurrentDiff = diff; bestDC = dc; }
    }
    return bestDC;
}

void reset_globals() {
    systemData.begin();
    mock_millis = 0; voltage_mv = 1000.0f; current_ma = 0.0f; mAh_charged = 0.0;
    dutyCycle = 0; chargingState = CHARGE_IDLE; chargeLog.clear();
    sim = BatterySim(); overtemp_trip_counter = 0; currentAppState = APP_STATE_IDLE;
    postModelAppState = APP_STATE_IDLE;
    currentIRState = IR_STATE_IDLE; isMeasuringResistance = false; isCharging = false;
    currentModel.isModelBuilt = false;
    mock_server.args.clear();
    homeScreen.begin();
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    mock_sample_count = 0;
    for (int i = 0; i < PLOT_WIDTH; i++) {
        temp1_values[i] = NAN;
        temp2_values[i] = NAN;
        diff_values[i] = NAN;
        voltage_values[i] = NAN;
        current_values[i] = NAN;
    }
    for (int i=0; i<ADC_CH_COUNT; i++) {
        systemData._lastSnapshots[i].sum = 0;
        systemData._lastSnapshots[i].count = 0;
    }
}

void test_model_accuracy() {
    std::cout << "Running test_model_accuracy..." << std::endl;
    reset_globals();
    currentAppState = APP_STATE_BUILDING_MODEL;
    buildModelPhase = BuildModelPhase::Idle;
    int safety_counter = 0;
    while (safety_counter++ < 1000000) {
        mock_millis += 10;
        mock_sample_count++;
        systemData.update(estimateCurrent(dutyCycle));
        if (currentAppState == APP_STATE_BUILDING_MODEL) {
            buildCurrentModelStep();
        } else {
            break;
        }
    }
    if (!currentModel.isModelBuilt) {
        std::cout << "FAILED: Model not built. Phase: " << (int)buildModelPhase << ", Points: " << mock_dutyCycles.size() << std::endl;
        assert(false);
    }
    float sum_sq_error = 0;
    int count = 0;
    for (int dc = 0; dc <= 255; dc += 10) {
        float actual = sim.getCurrent(dc);
        float estimated = estimateCurrent(dc);
        float error = actual - estimated;
        sum_sq_error += error * error;
        count++;
    }
    float rmse = std::sqrt(sum_sq_error / count);
    std::cout << "  RMSE: " << rmse << std::endl;
    assert(rmse < 0.05);

    // Test inverse mapping
    float target = 1.0f;
    int dc = estimateDutyCycleForCurrent(target);
    float est = estimateCurrent(dc);
    std::cout << "  Target 1.0A -> Duty " << dc << " -> Estimated " << est << "A" << std::endl;
    assert(std::abs(est - target) < 0.05);

    std::cout << "test_model_accuracy PASSED" << std::endl << std::endl;
}

void test_overtemp_shutdown() {
    std::cout << "Running test_overtemp_shutdown..." << std::endl;
    reset_globals();
    currentAppState = APP_STATE_CHARGING;
    chargingState = CHARGE_MONITOR;

    // Simulate battery heating up rapidly
    sim.temp = 50.0f;

    int loop_count = 0;
    while (loop_count++ < 1000) {
        mock_millis += CHARGE_EVALUATION_INTERVAL_MS;
        mock_sample_count++;
        systemData.update(estimateCurrent(dutyCycle));
        chargeBattery();
        if (chargingState == CHARGE_STOPPED) break;
    }

    assert(chargingState == CHARGE_STOPPED);
    std::cout << "  Shutdown triggered at millis: " << mock_millis << std::endl;
    std::cout << "test_overtemp_shutdown PASSED" << std::endl << std::endl;
}

void test_ir_measurement() {
    std::cout << "Running test_ir_measurement..." << std::endl;
    reset_globals();

    // Simulate web command "ir"
    WEB_LOCK();
    isMeasuringResistance = true;
    if (currentModel.isModelBuilt) {
        currentIRState = IR_STATE_START;
        currentAppState = APP_STATE_MEASURING_IR;
    } else {
        postModelAppState = APP_STATE_MEASURING_IR;
        buildModelPhase = BuildModelPhase::Idle;
        currentAppState = APP_STATE_BUILDING_MODEL;
    }
    WEB_UNLOCK();

    if (currentAppState == APP_STATE_BUILDING_MODEL) {
        std::cout << "  Correctly triggered model build before IR." << std::endl;
    }

    // For the rest of the test, let's manually build the model so IR measurement can proceed with valid current estimation if it needs it.
    // Actually, IR measurement seems to use ADC directly, but systemData.update overrides it if model is missing.

    int loop_count = 0;
    while (currentAppState != APP_STATE_IDLE && loop_count++ < 1000000) {
        mock_millis += 50;
        mock_sample_count++;
        sim.update(0.05f, (int)dutyCycle);
        systemData.update(estimateCurrent(dutyCycle));

        if (currentAppState == APP_STATE_BUILDING_MODEL) {
            buildCurrentModelStep();
        } else if (currentAppState == APP_STATE_MEASURING_IR) {
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) currentAppState = APP_STATE_IDLE;
        }
    }

    assert(currentAppState == APP_STATE_IDLE);
    assert(resistanceDataCountPairs > 0);
    std::cout << "  Pairs measured: " << resistanceDataCountPairs << ", Slope: " << regressedInternalResistancePairsSlope << ", Intercept: " << regressedInternalResistancePairsIntercept << std::endl;
    if (dutyCycle != 0) {
        std::cout << "  BUG REPRODUCED: dutyCycle is " << dutyCycle << " after IR measurement!" << std::endl;
    }
    assert(dutyCycle == 0);
    std::cout << "test_ir_measurement PASSED" << std::endl << std::endl;
}

void test_dead_region_detection() {
    std::cout << "Running test_dead_region_detection (Offset = 150mV)..." << std::endl;
    reset_globals();
    // Simulate a high offset in the simulator
    systemData.setCurrentZeroOffsetMv(150.0f);

    currentAppState = APP_STATE_BUILDING_MODEL;
    buildModelPhase = BuildModelPhase::Idle;

    int safety_counter = 0;
    while (safety_counter++ < 1000000) {
        mock_millis += 10;
        mock_sample_count++;
        systemData.update(estimateCurrent(dutyCycle));
        if (currentAppState == APP_STATE_BUILDING_MODEL) {
            buildCurrentModelStep();
        } else {
            break;
        }
    }

    assert(currentModel.isModelBuilt);
    std::cout << "  Calibrated Offset: " << systemData.getCurrentZeroOffsetMv() << " mV, Threshold: " << MEASURABLE_CURRENT_THRESHOLD << " A" << std::endl;
    std::cout << "test_dead_region_detection PASSED" << std::endl << std::endl;
}

void test_full_flow() {
    std::cout << "Running test_full_flow (Build Model -> Charge)..." << std::endl;
    reset_globals();

    // Simulate web command "charge"
    WEB_LOCK();
    resetAh = true;
    postModelAppState = APP_STATE_CHARGING;
    WEB_UNLOCK();
    buildModelPhase = BuildModelPhase::Idle;
    currentAppState = APP_STATE_BUILDING_MODEL;

    int loop_count = 0;
    while (loop_count++ < 400000) {
        mock_millis += 100;
        mock_sample_count++;
        sim.update(0.1f, (int)dutyCycle);
        systemData.update(estimateCurrent(dutyCycle));

        // Sync global mAh_charged with systemData
        mAh_charged = systemData.getData().mah_charged;

        if (currentAppState == APP_STATE_BUILDING_MODEL) {
            buildCurrentModelStep();
        } else if (currentAppState == APP_STATE_CHARGING) {
            if (mock_millis - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = mock_millis;
                chargeBattery();
            }
        }

        if (loop_count % 10000 == 0) {
            std::cout << "  Progress: loop=" << loop_count << ", State=" << (int)currentAppState << ", ChargingState=" << (int)chargingState << ", mAh=" << mAh_charged << ", Duty=" << dutyCycle << std::endl;
        }

        if (currentAppState == APP_STATE_CHARGING && chargingState == CHARGE_MONITOR && mAh_charged > 100) break;
        if (chargingState == CHARGE_STOPPED) break;
    }

    SystemData finalD = systemData.getData();
    std::cout << "  Final State: App=" << (int)currentAppState << ", Charging=" << (int)chargingState << ", mAh=" << finalD.mah_charged << std::endl;

    assert(currentAppState == APP_STATE_CHARGING);
    assert(chargingState == CHARGE_MONITOR);
    assert(finalD.mah_charged > 100);
    std::cout << "Successfully transitioned from model building to active charging." << std::endl;
    std::cout << "test_full_flow PASSED" << std::endl << std::endl;
}

void test_web_handlers() {
    std::cout << "Running test_web_handlers (API Migration check)..." << std::endl;
    reset_globals();
    AsyncWebServerRequest req;
    mock_server.lastResponseContent = "";
    handleData(&req);
    assert(mock_server.lastResponseCode == 410);
    std::cout << "  HTTP 410 Gone correctly returned for REST API." << std::endl;
    std::cout << "test_web_handlers PASSED" << std::endl << std::endl;
}

void test_websocket_communications() {
    std::cout << "Running test_websocket_communications..." << std::endl;
    reset_globals();
    ws.onEvent(handleWebSocketEvent);
    ws._mockClient.lastBinary.clear();

    // 1. Test Connection
    std::cout << "  Simulating client connection..." << std::endl;
    ws.mockConnect();
    // Connect triggers sendCborState and sendCborAmbient
    assert(ws._mockClient.lastBinary.size() > 0);
    std::cout << "  Initial data received size: " << ws._mockClient.lastBinary.size() << " bytes" << std::endl;

    // 2. Test Command
    std::cout << "  Simulating WebSocket command 'stop'..." << std::endl;
    currentAppState = APP_STATE_CHARGING;
    ws.mockReceiveText("stop");
    // In mock, processCommand is called.
    if (currentAppState != APP_STATE_IDLE) {
        std::cout << "  Warning: currentAppState is " << (int)currentAppState << ", expected " << (int)APP_STATE_IDLE << std::endl;
    }
    assert(currentAppState == APP_STATE_IDLE);
    assert(dutyCycle == 0);
    std::cout << "  Command 'stop' processed correctly." << std::endl;

    // 3. Test Data Request (History)
    std::cout << "  Requesting history via WebSocket..." << std::endl;
    ws._mockClient.lastBinary.clear();
    ws.mockReceiveText("REQ_HISTORY");
    assert(ws._mockClient.lastBinary.size() > 0);
    assert(ws._mockClient.lastBinary[0] == 0xA5); // Map(5)
    std::cout << "  History data received via WS." << std::endl;

    std::cout << "test_websocket_communications PASSED" << std::endl << std::endl;
}

void test_profiling_logic() {
    std::cout << "Running test_profiling_logic..." << std::endl;
    reset_globals();

    // Simulate a frame
    g_frameStartUs = 1000000;
    mock_esp_timer_now = 1000000;

    // Core 0 event
    recordEvent(0, 1, 100, 200);
    // Core 1 event
    recordEvent(1, 2, 500, 300);

    sendFramePacket(false);

    assert(ws.lastBinaryAll.size() >= 18 + 2*6);
    uint16_t magic = ws.lastBinaryAll[0] | (ws.lastBinaryAll[1] << 8);
    assert(magic == 0x5450); // "TP"

    uint8_t c0 = ws.lastBinaryAll[14];
    uint8_t c1 = ws.lastBinaryAll[15];
    assert(c0 == 1);
    assert(c1 == 1);

    std::cout << "  Profiling packet verified (Magic: 0x" << std::hex << magic << std::dec << ", C0: " << (int)c0 << ", C1: " << (int)c1 << ")" << std::endl;
    std::cout << "test_profiling_logic PASSED" << std::endl << std::endl;
}

void test_stress_web_requests() {
    std::cout << "Running test_stress_web_requests..." << std::endl;
    reset_globals();
    AsyncWebServerRequest req;

    // Simulate many rapid requests
    for (int i = 0; i < 100; i++) {
        mock_millis += 1;
        mock_sample_count++;
        systemData.update(estimateCurrent(dutyCycle));

        // Simulate AJAX state request
        req._args["type"] = "state";
        handleData(&req);
        // assert(mock_server.lastResponseContent.length() > 0);

        // Simulate AJAX history request
        req._args["type"] = "history";
        handleData(&req);

        // Simulate broadcast
        broadcastLiveTelemetry();

        // Simulate random command
        if (i % 20 == 0) {
            req._args["cmd"] = "reset";
            handleCommand(&req);
        }
    }

    std::cout << "  Processed 100 iterations of mixed requests successfully." << std::endl;
    std::cout << "test_stress_web_requests PASSED" << std::endl << std::endl;
}

int main() {
    test_model_accuracy();
    test_dead_region_detection();
    test_ir_measurement();
    test_overtemp_shutdown();
    test_full_flow();
    test_web_handlers();
    test_websocket_communications();
    test_profiling_logic();
    test_stress_web_requests();
    std::cout << "ALL TESTS PASSED!" << std::endl;
    return 0;
}
