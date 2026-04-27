#include "definitions.h"
#include "logging.h"
#include "graphing.h"
#include "charging.h"
#include "internal_resistance.h"
#include "home_screen.h"
#include "adc_dma.h"
#include "SystemDataManager.h"
#include "AdvancedPolynomialFitter.hpp"
#include <WiFi.h>
#ifndef MOCK_TEST
#include <ESPAsyncWebServer.h>
#endif

#include <vector>
#include <cmath>
#include <algorithm>
#include <ArduinoEigenDense.h> // needed for QR solve

// --- Constants / configuration ---

constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION_BITS) - 1;

// Replace repeated magic numbers with named constants when appropriate
constexpr TickType_t TASK_DELAY_SHT4_MS = 100;
constexpr TickType_t TASK_DELAY_THERMISTOR_MS = 50;

// --- External symbols ---
extern void measureInternalResistanceStep();
extern volatile IRState currentIRState;
extern void handleRoot(AsyncWebServerRequest *request);
extern void handleData(AsyncWebServerRequest *request);
extern void handleCommand(AsyncWebServerRequest *request);
#ifndef MOCK_TEST
extern void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
extern void broadcastLiveTelemetry();
#endif


// --- Application state ---
volatile AppState currentAppState = APP_STATE_IDLE;
DisplayState currentDisplayState = DISPLAY_STATE_IDLE;

// Shared measurement / UI globals
volatile float MEASURABLE_CURRENT_THRESHOLD = 0.005f;
volatile float voltage_mv = 1000.0f;
volatile float current_ma = 0.0f;
volatile double mAh_charged = 0.0;
volatile bool resetAh = false;
volatile uint32_t mAh_last_time = 0;

uint32_t dutyCycle = 0;

// Timers
unsigned long lastDataGatherTime = 0;
unsigned long lastChargingHouseTime = 0;

// PWM pin
constexpr int pwmPin = PWM_PIN;
double THERMISTOR_1_OFFSET = 0.0;

// Hardware / sensor objects
CurrentModel currentModel;
SHT4xSensor sht4Sensor;
HomeScreen homeScreen;
SystemDataManager systemData(sht4Sensor, THERMISTOR_PIN_1, THERMISTOR_VCC_PIN, THERMISTOR_1_OFFSET);

// Web Server
#ifndef MOCK_TEST
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
#endif

#ifndef MOCK_TEST
SemaphoreHandle_t webDataMutex = NULL;
#endif

// --- Profiling Globals ---
CoreBuf g_coreBuf[CORE_COUNT];
volatile uint32_t g_frameSeq = 0;
volatile uint32_t g_frameStartUs = 0;
volatile uint8_t g_frameFlags = 0;
TaskHandle_t g_masterTaskHandle = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// --- Build model state ---
volatile BuildModelPhase buildModelPhase = BuildModelPhase::Idle;
int buildModelDutyCycle = 0;
unsigned long buildModelLastStepTime = 0;
float calibrationSum = 0;
float calibrationMax = 0;
int calibrationCount = 0;
uint32_t lastKnownSampleCount = 0;
volatile float noiseFloorMv = 0;
std::vector<float> dutyCycles;
std::vector<float> currents;

// --- Profiling Helpers ---
void recordEvent(uint8_t core, uint8_t taskId, uint16_t startUs, uint16_t durUs, uint8_t flags) {
    if (core >= CORE_COUNT) return;
    portENTER_CRITICAL(&g_mux);
    uint8_t idx = g_coreBuf[core].count;
    if (idx < MAX_EVENTS_PER_CORE) {
        g_coreBuf[core].events[idx].taskId = taskId;
        g_coreBuf[core].events[idx].flags  = flags;
        g_coreBuf[core].events[idx].startUs = startUs;
        g_coreBuf[core].events[idx].durUs   = durUs;
        g_coreBuf[core].count = idx + 1;
    }
    portEXIT_CRITICAL(&g_mux);
}

static inline void putU8(uint8_t *buf, size_t &off, uint8_t v) {
    buf[off++] = v;
}

static inline void putU16LE(uint8_t *buf, size_t &off, uint16_t v) {
    buf[off++] = (uint8_t)(v & 0xFF);
    buf[off++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void putU32LE(uint8_t *buf, size_t &off, uint32_t v) {
    buf[off++] = (uint8_t)(v & 0xFF);
    buf[off++] = (uint8_t)((v >> 8) & 0xFF);
    buf[off++] = (uint8_t)((v >> 16) & 0xFF);
    buf[off++] = (uint8_t)((v >> 24) & 0xFF);
}

static void sendFramePacket(bool timeoutFlag) {
#ifndef MOCK_TEST
    if (ws.count() == 0) return;

    uint8_t packet[18 + (CORE_COUNT * MAX_EVENTS_PER_CORE * 6)];
    size_t p = 0;

    uint8_t c0, c1;
    portENTER_CRITICAL(&g_mux);
    c0 = g_coreBuf[0].count;
    c1 = g_coreBuf[1].count;
    portEXIT_CRITICAL(&g_mux);

    if (c0 > MAX_EVENTS_PER_CORE) c0 = MAX_EVENTS_PER_CORE;
    if (c1 > MAX_EVENTS_PER_CORE) c1 = MAX_EVENTS_PER_CORE;

    putU16LE(packet, p, 0x5450); // "TP"
    putU8(packet, p, 1);         // version
    putU8(packet, p, 2);         // cores
    putU32LE(packet, p, g_frameSeq);
    putU32LE(packet, p, g_frameStartUs);
    putU16LE(packet, p, (uint16_t)FRAME_PERIOD_US);
    putU8(packet, p, c0);
    putU8(packet, p, c1);

    uint8_t flags = 0;
    if (timeoutFlag) flags |= 0x01;
    if (c0 >= MAX_EVENTS_PER_CORE || c1 >= MAX_EVENTS_PER_CORE) flags |= 0x02;
    putU8(packet, p, flags);
    putU8(packet, p, 0);

    // Core 0 events
    for (uint8_t i = 0; i < c0; i++) {
        EventRec e;
        portENTER_CRITICAL(&g_mux);
        e = g_coreBuf[0].events[i];
        portEXIT_CRITICAL(&g_mux);

        putU8(packet, p, e.taskId);
        putU8(packet, p, e.flags);
        putU16LE(packet, p, e.startUs);
        putU16LE(packet, p, e.durUs);
    }

    // Core 1 events
    for (uint8_t i = 0; i < c1; i++) {
        EventRec e;
        portENTER_CRITICAL(&g_mux);
        e = g_coreBuf[1].events[i];
        portEXIT_CRITICAL(&g_mux);

        putU8(packet, p, e.taskId);
        putU8(packet, p, e.flags);
        putU16LE(packet, p, e.startUs);
        putU16LE(packet, p, e.durUs);
    }

    ws.binaryAll(packet, p);
#endif
}

static void masterTask(void *param) {
    (void)param;
    for (;;) {
        const uint32_t frameStart = (uint32_t)esp_timer_get_time();

        // Reset per-frame buffers.
        portENTER_CRITICAL(&g_mux);
        g_coreBuf[0].count = 0;
        g_coreBuf[1].count = 0;
        portEXIT_CRITICAL(&g_mux);

        g_frameStartUs = frameStart;
        g_frameSeq++;

        // In this integrated version, we don't have worker tasks that we notify.
        // Instead, the actual application tasks will call recordEvent during the frame.
        // We just wait for the next frame.

        vTaskDelay(pdMS_TO_TICKS(FRAME_PERIOD_US / 1000));

        sendFramePacket(false);
    }
}

// --- Utility helpers ---
void applyDuty(uint32_t duty) {
    dutyCycle = std::min<uint32_t>(duty, PWM_MAX_DUTY_CYCLE);
    analogWrite(pwmPin, static_cast<int>(dutyCycle));
}

void setAppState(AppState s) {
    WEB_LOCK();
    currentAppState = s;
    WEB_UNLOCK();
}

void setBuildModelPhase(BuildModelPhase p) {
    WEB_LOCK();
    buildModelPhase = p;
    WEB_UNLOCK();
}

// --- PWM setup ---
void setupPWM() {
    pinMode(pwmPin, OUTPUT);
    analogWriteResolution(pwmPin, PWM_RESOLUTION_BITS);
    // Use the overloaded version for ESP32
#ifndef MOCK_TEST
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
#else
    analogWriteFrequency(PWM_FREQUENCY);
#endif
    applyDuty(0);
}

// --- Thermistor / sensor helpers ---
void getThermistorReadings(double& temp1, double& temp2, double& tempDiff, float& t1_millivolts, float& voltage, float& current) {
    SystemData d = systemData.getData();
    temp1 = d.ambient_temp_c;
    temp2 = d.battery_temp_c;
    tempDiff = d.temp_diff_c;
    t1_millivolts = 0;
    voltage = d.battery_voltage_v;
    current = d.charge_current_a;
}

// --- Non-blocking build current model ---
void buildCurrentModelStep() {
    const unsigned long now = millis();
    static BuildModelPhase lastPhase = BuildModelPhase::Finish;

    if (buildModelPhase != lastPhase) {
        Serial.printf("DEBUG: buildCurrentModelStep phase transition: %d -> %d\n", (int)lastPhase, (int)buildModelPhase);
        lastPhase = buildModelPhase;
    }

    switch (buildModelPhase) {
        case BuildModelPhase::Idle:
            dutyCycles.clear();
            currents.clear();
            applyDuty(0);
            buildModelDutyCycle = 0;
            buildModelLastStepTime = now;
            setBuildModelPhase(BuildModelPhase::Settle);
            Serial.println("Building Current Model: Settling (2s)...");
            Serial.flush();
            break;
        case BuildModelPhase::Settle:
            if (now - buildModelLastStepTime >= 2000) {
                calibrationSum = 0;
                calibrationMax = 0;
                calibrationCount = 0;
                SystemData d = systemData.getData();
                lastKnownSampleCount = d.current_sample_count;
                buildModelLastStepTime = now; // Reset timer for Calibrate phase
                setBuildModelPhase(BuildModelPhase::Calibrate);
                Serial.println("Building Current Model: Calibrating Zero Offset...");
            }
            break;
        case BuildModelPhase::Calibrate:
            {
                SystemData d = systemData.getData();
                if (d.current_sample_count != lastKnownSampleCount) {
                    lastKnownSampleCount = d.current_sample_count;
                    calibrationSum += d.current_mv;
                    if (d.current_mv > calibrationMax) calibrationMax = d.current_mv;
                    calibrationCount++;
                    Serial.printf("  Calibrating... %d/200 samples (Current: %.2f mV, count=%u)\n", calibrationCount, d.current_mv, d.current_sample_count);
                }

                if (calibrationCount >= 200) {
                    float avgOffset = calibrationSum / calibrationCount;
                    systemData.setCurrentZeroOffsetMv(avgOffset);

                    WEB_LOCK();
                    noiseFloorMv = (calibrationMax - avgOffset) * 2.0f;
                    if (noiseFloorMv < 1.0f) noiseFloorMv = 1.0f;
                    WEB_UNLOCK();

                    Serial.printf("Auto-calibration complete. Offset: %.2f mV, NoiseFloor: %.2f mV\n", avgOffset, (float)noiseFloorMv);
                    buildModelDutyCycle = 1;
                    applyDuty(buildModelDutyCycle); // Start applying duty immediately
                    buildModelLastStepTime = now;
                    setBuildModelPhase(BuildModelPhase::DetectDeadRegion);
                    Serial.println("Building Current Model: Detecting Dead Region...");
                }

                if (now - buildModelLastStepTime > 10000) { // Increased timeout
                    Serial.printf("Auto-calibration TIMEOUT (Samples: %d) - fallback to default.\n", calibrationCount);
                    systemData.setCurrentZeroOffsetMv(CURRENT_SHUNT_PIN_ZERO_OFFSET);
                    WEB_LOCK();
                    noiseFloorMv = 5.0f;
                    WEB_UNLOCK();
                    buildModelDutyCycle = 1;
                    applyDuty(buildModelDutyCycle);
                    buildModelLastStepTime = now;
                    setBuildModelPhase(BuildModelPhase::DetectDeadRegion);
                }
            }
            break;
        case BuildModelPhase::DetectDeadRegion:
            if (now - buildModelLastStepTime >= 250) {
                SystemData d = systemData.getData();
                float currentMv = d.current_mv - systemData.getCurrentZeroOffsetMv();

                Serial.printf("  Detect: Duty %d, currentMv-Offset: %.2f mV (NoiseFloor: %.2f mV), I: %.4f A\n",
                              buildModelDutyCycle, currentMv, (float)noiseFloorMv, d.charge_current_a);

                if (currentMv > noiseFloorMv && d.charge_current_a > 0.001f) {
                    WEB_LOCK();
                    MEASURABLE_CURRENT_THRESHOLD = d.charge_current_a;
                    WEB_UNLOCK();
                    dutyCycles.push_back(0.0f);
                    currents.push_back(0.0f);
                    dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                    currents.push_back(d.charge_current_a);

                    Serial.printf("Dead region ends at Duty: %d, Threshold: %.3f A\n", buildModelDutyCycle, (float)MEASURABLE_CURRENT_THRESHOLD);
                    buildModelDutyCycle += 1;
                    buildModelLastStepTime = now; // Reset timer for SetDuty
                    setBuildModelPhase(BuildModelPhase::SetDuty);
                } else {
                    buildModelDutyCycle += 1; // Slightly faster increment
                    if (buildModelDutyCycle > MAX_DUTY_CYCLE) {
                        Serial.println("Error: Could not detect current above noise floor.");
                        applyDuty(0);
                        setAppState(APP_STATE_IDLE);
                        setBuildModelPhase(BuildModelPhase::Idle);
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
                setBuildModelPhase(BuildModelPhase::WaitMeasurement);
            } else {
                buildModelLastStepTime = now; // Reset timer for Finish
                setBuildModelPhase(BuildModelPhase::Finish);
            }
            break;
        case BuildModelPhase::WaitMeasurement:
            if (now - buildModelLastStepTime >= BUILD_CURRENT_MODEL_DELAY) {
                SystemData d = systemData.getData();
                if (d.charge_current_a >= MEASURABLE_CURRENT_THRESHOLD) {
                    dutyCycles.push_back(static_cast<float>(buildModelDutyCycle));
                    currents.push_back(d.charge_current_a);
                    Serial.printf("  Model point: Duty %d, Current %.3f A, Samples: %u\n",
                                  buildModelDutyCycle, d.charge_current_a, d.current_sample_count);
                }
                buildModelDutyCycle += 5;
                setBuildModelPhase(BuildModelPhase::SetDuty);
            }
            break;
        case BuildModelPhase::Finish:
            if (dutyCycles.size() >= 2) {
                const int degree = 3;
                AdvancedPolynomialFitter fitter;
                std::vector<float> coeffs = fitter.fitPolynomialLebesgue(dutyCycles, currents, degree);

                WEB_LOCK();
                currentModel.coefficients.resize(coeffs.size());
                for (size_t i = 0; i < coeffs.size(); ++i) {
                    currentModel.coefficients(i) = coeffs[i];
                }

                if (degree >= 0) currentModel.coefficients(0) = 0.0;
                currentModel.isModelBuilt = true;
                WEB_UNLOCK();

                applyDuty(0);
                setAppState(APP_STATE_IDLE);
                startCharging();
            } else {
                WEB_LOCK();
                currentModel.isModelBuilt = false;
                WEB_UNLOCK();
                applyDuty(0);
                setAppState(APP_STATE_IDLE);
            }
            setBuildModelPhase(BuildModelPhase::Idle);
            break;
    }
}

float estimateCurrent(int duty) {
    float result = 0.0f;
    WEB_LOCK();
    if (currentModel.isModelBuilt) {
        const float x = static_cast<float>(duty);
        double sum = 0.0;
        for (int i = 0; i < currentModel.coefficients.size(); ++i) {
            sum += currentModel.coefficients(i) * std::pow(x, i);
        }
        result = static_cast<float>(std::max(0.0, sum));
    }
    WEB_UNLOCK();
    return result;
}

int estimateDutyCycleForCurrent(float targetCurrent) {
    if (!currentModel.isModelBuilt) return 0;
    int bestDC = 0;
    float closestCurrentDiff = std::numeric_limits<float>::max();
    for (int dc = MIN_CHARGE_DUTY_CYCLE; dc <= MAX_CHARGE_DUTY_CYCLE; ++dc) {
        float estimated = estimateCurrent(dc);
        float diff = std::abs(estimated - targetCurrent);
        if (diff < closestCurrentDiff) {
            closestCurrentDiff = diff;
            bestDC = dc;
        }
    }
    return bestDC;
}

// --- Tasks ---
void task_readSHT4x(void* parameter) {
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
        sht4Sensor.read();
        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        recordEvent(0, 1, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHT4_MS));
    }
}

void task_processAdcDma(void* parameter) {
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
        processAdcDma();
        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        recordEvent(0, 2, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_updateSystemData(void* parameter) {
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
        float est = estimateCurrent(dutyCycle);
        systemData.update(est);
        SystemData d = systemData.getData();

        WEB_LOCK();
        if (resetAh) {
            systemData.resetMah();
            resetAh = false;
        }
        voltage_mv = d.battery_voltage_v * 1000.0f;
        current_ma = d.charge_current_a * 1000.0f;
        mAh_charged = d.mah_charged;
        WEB_UNLOCK();
        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        recordEvent(1, 3, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void task_webServer(void* parameter) {
    Serial.println("WebServer task started.");
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
#ifndef MOCK_TEST
        ws.cleanupClients();
        broadcastLiveTelemetry();
#endif
        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        recordEvent(1, 4, (uint16_t)t0, (uint16_t)(t1 - t0));

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Initialization ---
void setup() {
    Serial.begin(115200);

#ifndef MOCK_TEST
    webDataMutex = xSemaphoreCreateRecursiveMutex();
#endif

    WiFi.softAP("NiMH-WebCharger", "password123");
    Serial.print("Web Interface at: http://");
    Serial.println(WiFi.softAPIP());

#ifndef MOCK_TEST
    ws.onEvent(handleWebSocketEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, handleRoot);
    server.on("/data", HTTP_GET, handleData);
    server.on("/command", HTTP_GET, handleCommand);
    server.begin();
#endif

    for (int i = 0; i < PLOT_WIDTH; ++i) {
        temp1_values[i] = 25.0f;
        temp2_values[i] = 25.0f;
        diff_values[i] = 0.0f;
        voltage_values[i] = 1.0f;
        current_values[i] = 0.0f;
    }

    sht4Sensor.begin();
    homeScreen.begin();
    systemData.begin();
    setupAdcDma();

    xTaskCreatePinnedToCore(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_processAdcDma, "ADC_DMA", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_updateSystemData, "SYS_DATA", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_webServer, "WebServer", 16384, NULL, 1, NULL, 1);

    // Master on core 0, slightly higher priority.
    xTaskCreatePinnedToCore(
        masterTask,
        "master",
        4096,
        nullptr,
        3,
        &g_masterTaskHandle,
        0
    );

    setupPWM();
    Serial.println("System Ready.");
}

void gatherData() {
    double temp1 = 0.0, temp2 = 0.0, tempDiff = 0.0;
    float t1_millivolts = 0.0f, voltage = 0.0f, current = 0.0f;
    getThermistorReadings(temp1, temp2, tempDiff, t1_millivolts, voltage, current);
    updateTemperatureHistory(temp1, temp2, tempDiff, voltage, current);
}

void loop() {
    uint32_t frameRef = g_frameStartUs;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
    const unsigned long now = millis();

    switch (currentAppState) {
        case APP_STATE_IDLE: break;
        case APP_STATE_BUILDING_MODEL: buildCurrentModelStep(); break;
        case APP_STATE_MEASURING_IR:
            measureInternalResistanceStep();
            if (currentIRState == IR_STATE_IDLE) setAppState(APP_STATE_IDLE);
            break;
        case APP_STATE_CHARGING:
            if (now - lastChargingHouseTime >= CHARGING_HOUSEKEEP_INTERVAL) {
                lastChargingHouseTime = now;
                if (!chargeBattery()) setAppState(APP_STATE_IDLE);
            }
            break;
    }

    if (now - lastDataGatherTime >= PLOT_DATA_UPDATE_INTERVAL) {
        lastDataGatherTime = now;
        gatherData();
    }
    homeScreen.gatherData();

    // Give other tasks time to run
    uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
    recordEvent(1, 0, (uint16_t)t0, (uint16_t)(t1 - t0));

    vTaskDelay(10);
}
