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
volatile AppState postModelAppState = APP_STATE_IDLE;
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
volatile float estimatedTauThermal = 45.0f; // Default thermal time constant in seconds
volatile float estimatedTauSHT = 10.0f;    // SHT4x typical thermal response lag in seconds
volatile float estimatedTauTherm = 5.0f;   // Thermistor 1 typical thermal response lag in seconds
volatile float estimatedConvectiveH = DEFAULT_CONVECTIVE_H;
volatile float estimatedThermalResistance = 1.0f / (DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT / 45.0f);
volatile float estimatedThermalCapacitance = DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT;
volatile float estimatedThermalConductance = DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT / 45.0f;
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


/*
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
    if (client->queueLen() < 16) {
    ws.binaryAll(packet, p);}
    }
#endif
}

*/

#ifndef MOCK_TEST
// API Compatibility Helper: Overloaded functions to resolve different return types
// of ws.getClients() across varying ESPAsyncWebServer library versions (either pointers
// or object references) to a standard pointer before accessing client methods.
inline AsyncWebSocketClient* resolve_client(AsyncWebSocketClient* c) { return c; }
inline AsyncWebSocketClient* resolve_client(AsyncWebSocketClient& c) { return &c; }
#endif

static void sendFramePacket(bool timeoutFlag) {
#ifndef MOCK_TEST
    if (ws.count() == 0) return;

    // 1. Prepare the packet buffer
    uint8_t packet[18 + (CORE_COUNT * MAX_EVENTS_PER_CORE * 6)];
    size_t p = 0;

    // 2. Snapshot the counts and data quickly to minimize lock time
    uint8_t c0, c1;
    EventRec events0[MAX_EVENTS_PER_CORE];
    EventRec events1[MAX_EVENTS_PER_CORE];

    portENTER_CRITICAL(&g_mux);
    c0 = (g_coreBuf[0].count > MAX_EVENTS_PER_CORE) ? MAX_EVENTS_PER_CORE : g_coreBuf[0].count;
    c1 = (g_coreBuf[1].count > MAX_EVENTS_PER_CORE) ? MAX_EVENTS_PER_CORE : g_coreBuf[1].count;
    memcpy(events0, g_coreBuf[0].events, c0 * sizeof(EventRec));
    memcpy(events1, g_coreBuf[1].events, c1 * sizeof(EventRec));
    portEXIT_CRITICAL(&g_mux);

    // 3. Build the Header
    putU16LE(packet, p, 0x5450); // "TP"
    putU8(packet, p, 1);          // version
    putU8(packet, p, 2);          // cores
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

    // 4. Serialize Core 0
    for (uint8_t i = 0; i < c0; i++) {
        putU8(packet, p, events0[i].taskId);
        putU8(packet, p, events0[i].flags);
        putU16LE(packet, p, events0[i].startUs);
        putU16LE(packet, p, events0[i].durUs);
    }

    // 5. Serialize Core 1
    for (uint8_t i = 0; i < c1; i++) {
        putU8(packet, p, events1[i].taskId);
        putU8(packet, p, events1[i].flags);
        putU16LE(packet, p, events1[i].startUs);
        putU16LE(packet, p, events1[i].durUs);
    }

    // 6. Selective Broadcast (The Fix for the Disconnects)
    for (auto & c : ws.getClients()) {
        AsyncWebSocketClient* client = resolve_client(c);
        if (client->status() == WS_CONNECTED) {
            // Only send if the queue is not backed up.
            // 16 is a safe threshold for a 32-slot queue.
            if (client->queueLen() < 4) {
                client->binary(packet, p);
            } else {
                // Optional:
                Serial.printf("Skipping frame for client %u (Queue full)\n", client->id());
            }
        }
    }
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

        // Timing Precision Safety: Prevent truncation errors and tight loops if FRAME_PERIOD_US < 1000
        uint32_t delay_ms = (FRAME_PERIOD_US + 999) / 1000;
        if (delay_ms < 1) delay_ms = 1;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

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

// --- Thermal Characterize variables ---
static bool characterizationInitialized = false;
static double tempStart = 0.0;
static double tempAtShutoff = 0.0;
static unsigned long startTime = 0;
static unsigned long shutoffTime = 0;
static double peakTempAfterShutoff = 0.0;
static double peakAmbientTemp = 0.0;
static unsigned long peakTimeAfterShutoff = 0;
static int characPhase = 0; // 0: Heating, 1: Peak detection, 2: Cool-off
static unsigned long cooloffStartTime = 0;

struct ThermalSample {
    float t_rel_s;
    float tempDiff;
};
static std::vector<ThermalSample> cooloffSamples;
static unsigned long lastCooloffSampleTime = 0;
static double heatingPowerSum = 0.0;
static unsigned int heatingPowerCount = 0;

static const int CHARACTERIZATION_ITERATIONS = 3;
static int characIteration = 0;
static bool characSweepDone = false;
static unsigned long characSettleStartTime = 0;
static float sumTauThermal = 0.0f;
static float sumTauThermistor = 0.0f;
static float sumTauSHT4x = 0.0f;
static float sumConvectiveH = 0.0f;
static float sumThermalResistance = 0.0f;
static float sumThermalCapacitance = 0.0f;
static float sumThermalConductance = 0.0f;

static unsigned long currentHeatingDurationMs = 15000;
static unsigned long currentCooloffDurationMs = 60000;

static int sweepStep = -1;
static float sweepUnloadedV = 0.0f;
static float sweepUnloadedI = 0.0f;
static float sweepStepVoltInitial = 0.0f;
static float sweepVoltages[15] = {0.0f};
static float sweepCurrents[15] = {0.0f};
static int sweepDutyCycles[15] = {0};
static unsigned long sweepStepStartTime = 0;
static std::vector<DutyPair> sweepCatPairs;

// Transient recording buffer for thermal characterize sweep step 0
static constexpr size_t SWEEP_TRANSIENT_MAX_SAMPLES = 64;
static float sweep_transient_time_s[SWEEP_TRANSIENT_MAX_SAMPLES];
static float sweep_transient_voltage_V[SWEEP_TRANSIENT_MAX_SAMPLES];
static float sweep_transient_current_A[SWEEP_TRANSIENT_MAX_SAMPLES];
static size_t sweep_transient_count = 0;
static unsigned long sweep_transient_start_ms = 0;
static unsigned long sweep_last_sample_ms = 0;

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

            // Reset thermal characterize variables
            characterizationInitialized = false;
            tempStart = 0.0;
            tempAtShutoff = 0.0;
            startTime = 0;
            shutoffTime = 0;
            peakTempAfterShutoff = 0.0;
            peakAmbientTemp = 0.0;
            peakTimeAfterShutoff = 0;
            characPhase = 0;
            cooloffStartTime = 0;
            characIteration = 0;
            sumTauThermal = 0.0f;
            sumTauThermistor = 0.0f;
            sumTauSHT4x = 0.0f;
            sumConvectiveH = 0.0f;
            sumThermalResistance = 0.0f;
            sumThermalCapacitance = 0.0f;
            sumThermalConductance = 0.0f;
            currentHeatingDurationMs = 15000;
            currentCooloffDurationMs = 60000;
            cooloffSamples.clear();
            lastCooloffSampleTime = 0;
            heatingPowerSum = 0.0;
            heatingPowerCount = 0;

            characSweepDone = false;
            characSettleStartTime = 0;

            sweepStep = -1;
            sweepUnloadedV = 0.0f;
            sweepUnloadedI = 0.0f;
            sweepStepVoltInitial = 0.0f;
            for (int k = 0; k < 15; k++) {
                sweepVoltages[k] = 0.0f;
                sweepCurrents[k] = 0.0f;
                sweepDutyCycles[k] = 0;
            }
            sweepStepStartTime = 0;
            sweepCatPairs.clear();

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
        case BuildModelPhase::ThermalCharacterize:
            {
                if (!characSweepDone) {
                    if (!characterizationInitialized) {
                        double t1, t2, td; float tmv, v, c;
                        getThermistorReadings(t1, t2, td, tmv, v, c);
                        startTime = now;
                        sweepStep = -1;
                        sweepStepStartTime = now;
                        applyDuty(0);
                        generateCategorizedDutyPairs(sweepCatPairs, 15);
                        characterizationInitialized = true;
                        characPhase = 0;
                        Serial.printf("Starting categorized %d-pair IR sweep before thermal characterization...\n", (int)sweepCatPairs.size());
                    }
                } else if (characPhase == 3) {
                    // Thermal Settling Phase: ensure battery thermally settles after IR sweep before starting Thermal Run 1
                    applyDuty(0);
                    if (now - characSettleStartTime >= 5000) {
                        characPhase = 0;
                        characIteration = 0;
                        characterizationInitialized = false;
                        sumTauThermal = 0.0f;
                        sumTauThermistor = 0.0f;
                        sumTauSHT4x = 0.0f;
                        sumConvectiveH = 0.0f;
                        sumThermalResistance = 0.0f;
                        sumThermalCapacitance = 0.0f;
                        sumThermalConductance = 0.0f;
                        currentHeatingDurationMs = 15000;
                        currentCooloffDurationMs = 60000;
                        Serial.println("  Thermal Settling Complete. Starting 3-iteration thermal characterization...");
                    }
                    break;
                } else {
                    if (!characterizationInitialized) {
                        double t1, t2, td; float tmv, v, c;
                        getThermistorReadings(t1, t2, td, tmv, v, c);
                        tempStart = t2;
                        startTime = now;
                        shutoffTime = 0;
                        peakTempAfterShutoff = 0.0;
                        peakAmbientTemp = t1;
                        peakTimeAfterShutoff = 0;
                        characPhase = 0;
                        cooloffStartTime = 0;
                        cooloffSamples.clear();
                        lastCooloffSampleTime = 0;
                        heatingPowerSum = 0.0;
                        heatingPowerCount = 0;
                        characterizationInitialized = true;

                        float targetI = 0.90f * estimateCurrent(MAX_DUTY_CYCLE);
                        int characDuty = estimateDutyCycleForCurrent(targetI);
                        if (characDuty < MIN_CHARGE_DUTY_CYCLE) characDuty = MIN_CHARGE_DUTY_CYCLE;
                        applyDuty(characDuty);
                        Serial.printf("Thermal Characterize [Run %d/%d] Phase 1 (Heating, %lu ms): applied 90%% load (Duty %d, Target %.3f A), initial temp: %.2f C\n",
                                      characIteration + 1, CHARACTERIZATION_ITERATIONS, currentHeatingDurationMs, characDuty, targetI, tempStart);
                    }
                }

                if (characPhase == 0) {
                    if (!characSweepDone) {
                        // Categorized sweep state machine with dynamic adaptive delays and electrode evaluation
                        unsigned long requiredDelay = g_electrode.adaptiveDelayMs > 0 ? (unsigned long)g_electrode.adaptiveDelayMs : 1000UL;

                        if (sweepStep == -1) {
                            if (now - startTime >= requiredDelay) {
                                double t1, t2, td; float tmv, v, c;
                                getThermistorReadings(t1, t2, td, tmv, v, c);
                                sweepUnloadedV = v;
                                sweepUnloadedI = c;
                                sweepStep = 0;

                                int targetDuty = 0;
                                if (!sweepCatPairs.empty()) {
                                    targetDuty = sweepCatPairs[0].highDC;
                                } else {
                                    float maxC = estimateCurrent(MAX_DUTY_CYCLE);
                                    float minC = MEASURABLE_CURRENT_THRESHOLD > 0.01f ? MEASURABLE_CURRENT_THRESHOLD : 0.05f;
                                    targetDuty = estimateDutyCycleForCurrent(minC);
                                }
                                float slope = 0.0f;
                                if (!isDutyCycleLinearRegion(targetDuty, slope)) {
                                    int minLinear = MIN_DUTY_CYCLE_START;
                                    while (minLinear < MAX_DUTY_CYCLE && !isDutyCycleLinearRegion(minLinear, slope)) minLinear++;
                                    targetDuty = minLinear;
                                }

                                applyDuty(targetDuty);
                                sweepStepStartTime = now;
                                sweepStepVoltInitial = v;
                                sweepDutyCycles[0] = targetDuty;

                                // Initialize transient buffer for sweep step 0
                                sweep_transient_count = 0;
                                sweep_transient_start_ms = now;
                                sweep_last_sample_ms = 0;

                                Serial.printf("  Sweep Step 1/%d: Applied Duty %d (unloadedV = %.3f V, delay %lu ms)\n",
                                              (int)(sweepCatPairs.empty() ? 15 : sweepCatPairs.size()), targetDuty, sweepUnloadedV, requiredDelay);
                            }
                        } else if (sweepStep >= 0 && sweepStep < (int)(sweepCatPairs.empty() ? 15 : sweepCatPairs.size())) {
                            // Record transient data during step 0
                            if (sweepStep == 0 && sweep_transient_count < SWEEP_TRANSIENT_MAX_SAMPLES) {
                                if (sweep_last_sample_ms == 0 || now - sweep_last_sample_ms >= 10) {
                                    double t1, t2, td; float tmv, v, c;
                                    getThermistorReadings(t1, t2, td, tmv, v, c);
                                    sweep_transient_time_s[sweep_transient_count] = (now - sweep_transient_start_ms) * 0.001f;
                                    sweep_transient_voltage_V[sweep_transient_count] = v;
                                    sweep_transient_current_A[sweep_transient_count] = c;
                                    sweep_transient_count++;
                                    sweep_last_sample_ms = now;
                                }
                            }

                            if (now - sweepStepStartTime >= requiredDelay) {
                                double t1, t2, td; float tmv, v, c;
                                getThermistorReadings(t1, t2, td, tmv, v, c);
                                sweepVoltages[sweepStep] = v;
                                sweepCurrents[sweepStep] = c;

                                // Perform transient electrode evaluation on step 0
                                if (sweepStep == 0) {
                                    evaluateElectrodeParameters(
                                        sweep_transient_time_s,
                                        sweep_transient_voltage_V,
                                        sweep_transient_current_A,
                                        sweep_transient_count,
                                        sweepUnloadedV,
                                        sweepUnloadedI
                                    );
                                }

                                int totalSteps = (int)(sweepCatPairs.empty() ? 15 : sweepCatPairs.size());
                                if (sweepStep < totalSteps - 1) {
                                    sweepStep++;
                                    int nextDuty = 0;
                                    if (!sweepCatPairs.empty()) {
                                        nextDuty = sweepCatPairs[sweepStep].highDC;
                                    } else {
                                        float maxC = estimateCurrent(MAX_DUTY_CYCLE);
                                        float minC = MEASURABLE_CURRENT_THRESHOLD > 0.01f ? MEASURABLE_CURRENT_THRESHOLD : 0.05f;
                                        float limitC = 0.90f * maxC;
                                        if (limitC <= minC) limitC = maxC;
                                        float targetI = minC + (float)sweepStep * (limitC - minC) / 14.0f;
                                        nextDuty = estimateDutyCycleForCurrent(targetI);
                                    }
                                    float slope = 0.0f;
                                    if (!isDutyCycleLinearRegion(nextDuty, slope)) {
                                        int minLinear = MIN_DUTY_CYCLE_START;
                                        while (minLinear < MAX_DUTY_CYCLE && !isDutyCycleLinearRegion(minLinear, slope)) minLinear++;
                                        nextDuty = minLinear;
                                    }
                                    sweepDutyCycles[sweepStep] = nextDuty;
                                    applyDuty(nextDuty);
                                    sweepStepStartTime = now;
                                    Serial.printf("  Sweep Step %d/%d: Applied Duty %d\n", sweepStep + 1, totalSteps, nextDuty);
                                } else {
                                    // Finished all sweep points: Evaluate, filter, and store valid pairs
                                    int validCount = 0;
                                    float lastValidI = 0.0f;
                                    float lastValidIR = STRUCTURED_IR_SWEEP_DEFAULT_FALLBACK;

                                    for (int k = 0; k < totalSteps; k++) {
                                        int lowDC = (!sweepCatPairs.empty() && k < (int)sweepCatPairs.size()) ? sweepCatPairs[k].lowDC : 0;
                                        int highDC = sweepDutyCycles[k];
                                        float v1 = sweepUnloadedV;
                                        float v2 = sweepVoltages[k];
                                        float i1 = sweepUnloadedI;
                                        float i2 = sweepCurrents[k];

                                        float corrI = 0.0f, corrIR = 0.0f;
                                        bool valid = evaluateAndCorrectPairData(lowDC, highDC, v1, v2, i1, i2, corrI, corrIR);
                                        if (valid && corrIR >= MIN_VALID_RESISTANCE && corrIR <= STRUCTURED_IR_SWEEP_MAX_LIMIT) {
                                            WEB_LOCK();
                                            if (!sweepCatPairs.empty() && sweepCatPairs[k].type == PAIR_TYPE_GLOBAL) {
                                                storeOrAverageResistanceData(corrI, corrIR, internalResistanceDataPairs, resistanceDataCountPairs);
                                            } else {
                                                storeOrAverageResistanceData(corrI, corrIR, internalResistanceData, resistanceDataCount);
                                            }
                                            WEB_UNLOCK();
                                            validCount++;
                                            lastValidI = corrI;
                                            lastValidIR = corrIR;
                                        }
                                    }

                                    WEB_LOCK();
                                    if (validCount > 0) {
                                        regressedInternalResistancePairsIntercept = lastValidIR;
                                        regressedInternalResistanceIntercept = lastValidIR;
                                    } else {
                                        regressedInternalResistancePairsIntercept = STRUCTURED_IR_SWEEP_DEFAULT_FALLBACK;
                                        regressedInternalResistanceIntercept = STRUCTURED_IR_SWEEP_DEFAULT_FALLBACK;
                                        storeOrAverageResistanceData(0.10f, STRUCTURED_IR_SWEEP_DEFAULT_FALLBACK, internalResistanceData, resistanceDataCount);
                                    }
                                    WEB_UNLOCK();

                                    tempAtShutoff = t2;
                                    peakTempAfterShutoff = t2;
                                    peakAmbientTemp = t1;
                                    peakTimeAfterShutoff = now;
                                    shutoffTime = now;
                                    applyDuty(0);
                                    characSweepDone = true;
                                    characSettleStartTime = now;
                                    characPhase = 3; // Thermal settling phase
                                    Serial.printf("  Sweep IR Evaluation Complete: %d/%d valid points, IR = %.4f Ohms. Transitioning to Thermal Settling Phase.\n",
                                                  validCount, totalSteps, regressedInternalResistancePairsIntercept);
                                }
                            }
                        }
                    } else {
                        // Sample power during heating phase
                        double t1_h, t2_h, td_h; float tmv_h, v_h, c_h;
                        getThermistorReadings(t1_h, t2_h, td_h, tmv_h, v_h, c_h);
                        heatingPowerSum += (v_h * c_h);
                        heatingPowerCount++;

                        if (now - startTime >= currentHeatingDurationMs) {
                            tempAtShutoff = t2_h;
                            peakTempAfterShutoff = t2_h;
                            peakAmbientTemp = t1_h;
                            peakTimeAfterShutoff = now;
                            shutoffTime = now;
                            applyDuty(0); // Shutoff load to observe sensor lag peak
                            characPhase = 1;
                            Serial.printf("Thermal Characterize [Iteration %d/3] Phase 2 (Overshoot Peak Detection): shutoff load at temp: %.4f C\n",
                                          characIteration + 1, tempAtShutoff);
                        }
                    }
                } else if (characPhase == 1) {
                    double t1, t2, td; float tmv, v, c;
                    getThermistorReadings(t1, t2, td, tmv, v, c);
                    static int consecutiveDeclineCount = 0;

                    if (t2 > peakTempAfterShutoff + 0.005) {
                        peakTempAfterShutoff = t2;
                        peakAmbientTemp = t1;
                        peakTimeAfterShutoff = now;
                        consecutiveDeclineCount = 0;
                    } else if (t2 < peakTempAfterShutoff - 0.005) {
                        consecutiveDeclineCount++;
                    } else {
                        consecutiveDeclineCount = 0;
                    }

                    // Proceed to dedicated cool-off phase when temperature consistently declines (4 consecutive steps) or we timeout (30s)
                    bool tempDeclined = (consecutiveDeclineCount >= 4);
                    bool timeout = (now - shutoffTime >= 30000);

                    if (tempDeclined || timeout) {
                        characPhase = 2;
                        cooloffStartTime = now;
                        consecutiveDeclineCount = 0;
                        Serial.printf("Thermal Characterize [Iteration %d/3] Phase 3 (Cool-off, %lu ms): Peak at %.4f C (lag: %lu ms). Starting cool-off.\n",
                                      characIteration + 1, currentCooloffDurationMs, peakTempAfterShutoff, peakTimeAfterShutoff - shutoffTime);
                    }
                } else if (characPhase == 2) {
                    // Let temperature decay to fit battery thermal inertia (estimatedTauThermal) cleanly
                    applyDuty(0); // Guarantee zero load

                    // Sample cool-off time series every 200ms
                    if (now - lastCooloffSampleTime >= 200) {
                        double t1_c, t2_c, td_c; float tmv_c, v_c, c_c;
                        getThermistorReadings(t1_c, t2_c, td_c, tmv_c, v_c, c_c);
                        float rel_t_s = (float)(now - peakTimeAfterShutoff) / 1000.0f;
                        float diff = (float)(t2_c - t1_c);
                        if (diff > 0.001f && rel_t_s >= 0.0f) {
                            cooloffSamples.push_back({rel_t_s, diff});
                        }
                        lastCooloffSampleTime = now;
                    }

                    if (now - cooloffStartTime >= currentCooloffDurationMs) {
                        double t1, t2, td; float tmv, v, c;
                        getThermistorReadings(t1, t2, td, tmv, v, c);
                        double tempEnd = t2;

                        // Log-linear regression over entire cool-off time series samples:
                        // ln(T_b - T_a) = ln(A) - t / tau -> tau = -1 / slope
                        double computedTau = 300.0; // Fallback
                        if (cooloffSamples.size() >= 5) {
                            double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                            size_t n = 0;
                            for (const auto& s : cooloffSamples) {
                                if (s.tempDiff > 0.002f) {
                                    double x = s.t_rel_s;
                                    double y = log((double)s.tempDiff);
                                    sumX += x;
                                    sumY += y;
                                    sumXY += x * y;
                                    sumX2 += x * x;
                                    n++;
                                }
                            }
                            if (n >= 5) {
                                double denom = (n * sumX2 - sumX * sumX);
                                if (std::abs(denom) > 1e-9) {
                                    double slope = (n * sumXY - sumX * sumY) / denom;
                                    if (slope < -1e-5) {
                                        computedTau = -1.0 / slope;
                                    }
                                }
                            }
                        }
                        if (computedTau < 10.0) computedTau = 10.0;
                        if (computedTau > 450.0) computedTau = 450.0;

                        // Solve sensor pole tau_s for two-pole thermal system where t_peak = (tau_b * tau_s / (tau_b - tau_s)) * ln(tau_b / tau_s)
                        double obsPeakDelayS = (peakTimeAfterShutoff > shutoffTime) ? (double)(peakTimeAfterShutoff - shutoffTime) / 1000.0 : 1.0;
                        if (obsPeakDelayS < 0.2) obsPeakDelayS = 0.2;

                        double tau_b = computedTau;
                        double low_s = 0.1, high_s = std::min(15.0, 0.45 * tau_b);
                        for (int iter = 0; iter < 25; iter++) {
                            double mid_s = 0.5 * (low_s + high_s);
                            double theoPeak = (tau_b * mid_s / (tau_b - mid_s)) * log(tau_b / mid_s);
                            if (theoPeak < obsPeakDelayS) {
                                low_s = mid_s;
                            } else {
                                high_s = mid_s;
                            }
                        }
                        double computedTauTherm = 0.5 * (low_s + high_s);
                        if (computedTauTherm < 0.5) computedTauTherm = 0.5;
                        if (computedTauTherm > 10.0) computedTauTherm = 10.0;

                        // SHT4x typical sensor lag
                        double computedTauSHT = computedTauTherm * 2.0;
                        if (computedTauSHT < 1.0) computedTauSHT = 1.0;
                        if (computedTauSHT > 20.0) computedTauSHT = 20.0;

                        // Characterize conductance and heat capacity from measured electrical heating power and temperature rise
                        float computedGTotal = 0.0f;
                        float computedCTheta = 0.0f;
                        float deltaTempHeat = (float)(tempAtShutoff - tempStart);
                        float heatDurationS = (shutoffTime > startTime) ? (float)(shutoffTime - startTime) / 1000.0f : 0.0f;
                        float avgHeatingP = (heatingPowerCount > 0) ? (float)(heatingPowerSum / heatingPowerCount) : 0.0f;

                        if (avgHeatingP > 0.01f && deltaTempHeat > 0.05f && heatDurationS > 1.0f) {
                            computedGTotal = (avgHeatingP / deltaTempHeat) * (1.0f - expf(-heatDurationS / (float)computedTau));
                            computedCTheta = computedGTotal * (float)computedTau;
                        } else {
                            computedCTheta = DEFAULT_CELL_MASS_KG * DEFAULT_SPECIFIC_HEAT;
                            computedGTotal = computedCTheta / std::max(1.0f, (float)computedTau);
                        }
                        if (computedGTotal < 0.001f) computedGTotal = 0.001f;
                        if (computedCTheta < 1.0f) computedCTheta = 1.0f;

                        float computedRTheta = 1.0f / computedGTotal;
                        float ambK = (float)peakAmbientTemp + 273.15f;
                        float G_rad = 4.0f * DEFAULT_EMISSIVITY * STEFAN_BOLTZMANN * DEFAULT_SURFACE_AREA_M2 * powf(ambK, 3.0f);
                        float G_conv = std::max(0.0001f, computedGTotal - G_rad);
                        float computedConvectiveH = G_conv / DEFAULT_SURFACE_AREA_M2;

                        sumTauThermal += (float)computedTau;
                        sumTauThermistor += (float)computedTauTherm;
                        sumTauSHT4x += (float)computedTauSHT;
                        sumConvectiveH += computedConvectiveH;
                        sumThermalResistance += computedRTheta;
                        sumThermalCapacitance += computedCTheta;
                        sumThermalConductance += computedGTotal;

                        Serial.printf("Iteration %d Complete: TauThermal = %.2f s, TauThermistor = %.2f s, TauSHT = %.2f s, ConvectiveH = %.4f W/(m^2 K), R_theta = %.2f K/W, C_theta = %.2f J/K, G_total = %.6f W/K\n",
                                      characIteration + 1, (float)computedTau, (float)computedTauTherm, (float)computedTauSHT,
                                      computedConvectiveH, computedRTheta, computedCTheta, computedGTotal);

                        characIteration++;
                        characterizationInitialized = false; // Trigger restart of heating phase for next iteration

                        if (characIteration < CHARACTERIZATION_ITERATIONS) {
                            // Self-Tuning: Adjust next cycle Heating and Cool-off durations to the newly inferred result!
                            currentHeatingDurationMs = (unsigned long)(computedTau * 0.8f * 1000.0f);
                            if (currentHeatingDurationMs < 8000) currentHeatingDurationMs = 8000;
                            if (currentHeatingDurationMs > 60000) currentHeatingDurationMs = 60000;

                            // Set cooloff duration target to 2.5 * Tau (clamped between 45s and 180s) to guarantee >90% decay dynamic range
                            currentCooloffDurationMs = (unsigned long)(computedTau * 2.5f * 1000.0f);
                            if (currentCooloffDurationMs < 45000) currentCooloffDurationMs = 45000;
                            if (currentCooloffDurationMs > 180000) currentCooloffDurationMs = 180000;

                            characPhase = 0;
                        } else {
                            // Average results across all completed iterations
                            float countDiv = (float)CHARACTERIZATION_ITERATIONS;
                            WEB_LOCK();
                            estimatedTauThermal = sumTauThermal / countDiv;
                            estimatedTauTherm = sumTauThermistor / countDiv;
                            estimatedTauSHT = sumTauSHT4x / countDiv;
                            estimatedConvectiveH = sumConvectiveH / countDiv;
                            estimatedThermalResistance = sumThermalResistance / countDiv;
                            estimatedThermalCapacitance = sumThermalCapacitance / countDiv;
                            estimatedThermalConductance = sumThermalConductance / countDiv;

                            applyDuty(0);
                            if (postModelAppState == APP_STATE_CHARGING) {
                                setAppState(APP_STATE_CHARGING);
                                startCharging();
                            } else if (postModelAppState == APP_STATE_MEASURING_IR) {
                                currentIRState = IR_STATE_START;
                                setAppState(APP_STATE_MEASURING_IR);
                            } else {
                                setAppState(APP_STATE_IDLE);
                            }
                            postModelAppState = APP_STATE_IDLE;
                            WEB_UNLOCK();

                            Serial.printf("Multi-Iteration Thermal Characterize Complete (All 3 Iterations Verified):\n");
                            Serial.printf("  Estimated Tau Thermal: %.2f s (Fitted & Averaged)\n", (float)estimatedTauThermal);
                            Serial.printf("  Estimated Tau Thermistor: %.2f s\n", (float)estimatedTauTherm);
                            Serial.printf("  Estimated Tau SHT4x: %.2f s\n", (float)estimatedTauSHT);
                            Serial.printf("  Estimated Convective H: %.4f W/(m^2 K)\n", (float)estimatedConvectiveH);
                            Serial.printf("  Estimated Thermal Resistance: %.2f K/W\n", (float)estimatedThermalResistance);
                            Serial.printf("  Estimated Thermal Capacitance: %.2f J/K\n", (float)estimatedThermalCapacitance);
                            Serial.printf("  Estimated Thermal Conductance: %.6f W/K\n", (float)estimatedThermalConductance);

                            characIteration = 0;
                            characPhase = 0;
                            buildModelLastStepTime = now;
                            setBuildModelPhase(BuildModelPhase::Idle);
                        }
                    }
                }
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
                // Mathematically consistent constrained fit: enforce zero current at zero duty
                std::vector<float> coeffs = fitter.fitPolynomialLebesgueConstrainedZero(dutyCycles, currents, degree);

                WEB_LOCK();
                currentModel.coefficients.resize(coeffs.size());
                for (size_t i = 0; i < coeffs.size(); ++i) {
                    currentModel.coefficients(i) = coeffs[i];
                }

                currentModel.isModelBuilt = true;
                applyDuty(0);
                WEB_UNLOCK();

                Serial.println("Duty Cycle Model Built. Transitioning to Thermal Characterization.");
                buildModelLastStepTime = now;
                setBuildModelPhase(BuildModelPhase::ThermalCharacterize);
            } else {
                WEB_LOCK();
                currentModel.isModelBuilt = false;
                setAppState(APP_STATE_IDLE);
                postModelAppState = APP_STATE_IDLE;
                WEB_UNLOCK();
                applyDuty(0);
                setBuildModelPhase(BuildModelPhase::Idle);
            }
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
        recordEvent(0, 1, (uint16_t)t0, (uint16_t)(t1 - t0), 0);

        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHT4_MS));
    }
}

void task_processAdcDma(void* parameter) {
    while (true) {
        uint32_t frameRef = g_frameStartUs;
        uint32_t t0 = (uint32_t)(esp_timer_get_time() - frameRef);
        processAdcDma();
        uint32_t t1 = (uint32_t)(esp_timer_get_time() - frameRef);
        recordEvent(0, 2, (uint16_t)t0, (uint16_t)(t1 - t0), 0);

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
        recordEvent(1, 3, (uint16_t)t0, (uint16_t)(t1 - t0), 0);

        vTaskDelay(pdMS_TO_TICKS(10));
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
        recordEvent(1, 4, (uint16_t)t0, (uint16_t)(t1 - t0), 0);

        vTaskDelay(pdMS_TO_TICKS(500));
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

    xTaskCreatePinnedToCore(task_readSHT4x, "SHT4", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_processAdcDma, "ADC_DMA", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task_updateSystemData, "SYS_DATA", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(task_webServer, "WebServer", 4096, NULL, 1, NULL, 1);

    // Master on core 0, slightly higher priority.
    xTaskCreatePinnedToCore(
        masterTask,
        "master",
        4096,
        nullptr,
        1,
        &g_masterTaskHandle,
        1
    );

    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    setupPWM();
    WiFi.setSleep(false); // prevent modem sleep to stay snappy

    Serial.println("System Ready. Boot pin setup on GPIO 0, LED on GPIO 2.");
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

    // --- Boot Pin Control (GPIO 0 Start Charging Trigger) ---
    static bool lastBootPinReading = HIGH;
    static unsigned long lastBootDebounceTime = 0;
    static bool bootButtonState = HIGH;

    int currentBootReading = digitalRead(BOOT_PIN);
    if (currentBootReading != lastBootPinReading) {
        lastBootDebounceTime = now;
        lastBootPinReading = currentBootReading;
    }

    if ((now - lastBootDebounceTime) > 50) {
        if (currentBootReading != bootButtonState) {
            bootButtonState = currentBootReading;
            if (bootButtonState == LOW) { // Button pressed (active LOW)
                Serial.println("Hardware BOOT button pressed: Triggering charging flow...");
                WEB_LOCK();
                resetAh = true;
                postModelAppState = APP_STATE_CHARGING;
                WEB_UNLOCK();
                setBuildModelPhase(BuildModelPhase::Idle);
                setAppState(APP_STATE_BUILDING_MODEL);
            }
        }
    }

    // --- Built-in LED Visual Feedback (GPIO 2) ---
    static unsigned long lastLedToggleTime = 0;
    static bool ledState = LOW;
    unsigned long ledInterval = 0;

    if (currentAppState == APP_STATE_BUILDING_MODEL || currentAppState == APP_STATE_MEASURING_IR) {
        ledInterval = 100; // Fast blink for model building & IR sweep
    } else if (currentAppState == APP_STATE_CHARGING) {
        if (chargingState == CHARGE_PULSE_ACTIVE) {
            ledInterval = 500; // Normal active pulse charging heartbeat
        } else if (chargingState == CHARGE_PULSE_IR_TEST || chargingState == CHARGE_PULSE_IR_REMEASURE) {
            ledInterval = 250; // IR pulse re-measurement testing
        } else {
            ledInterval = 0; // OFF when stopped
        }
    } else {
        ledInterval = 0; // OFF in IDLE
    }

    if (ledInterval == 0) {
        if (ledState != LOW) {
            ledState = LOW;
            digitalWrite(LED_PIN, LOW);
        }
    } else {
        if (now - lastLedToggleTime >= ledInterval) {
            lastLedToggleTime = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        }
    }

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
    recordEvent(1, 0, (uint16_t)t0, (uint16_t)(t1 - t0), 0);

    vTaskDelay(pdMS_TO_TICKS(10));
}
