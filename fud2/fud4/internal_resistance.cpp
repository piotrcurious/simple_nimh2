#include "internal_resistance.h"
#include "definitions.h"
#include "AdvancedPolynomialFitter.hpp"
#include <algorithm> // for nth_element
#include <cmath>


// External functions
extern void getThermistorReadings(double& temp1, double& temp2, double& tempDiff,
                                   float& t1_millivolts, float& voltage, float& current);

// State machine variables
volatile IRState currentIRState = IR_STATE_IDLE;
IRState nextIRState = IR_STATE_IDLE;
unsigned long irStateChangeTime = 0;
MeasurementData currentMeasurement;

// Duty cycle search variables
int minimalDutyCycle = 0;
int findMinDcLow = 0;
int findMinDcHigh = 0;
int findMinDcMid = 0;

// Pair generation variables
std::vector<std::pair<int, int>> dutyCyclePairs;
int pairIndex = 0;
int pairGenerationStep = 0;
int pairGenerationSubStep = 0;
int lowDc = 0;
int previousHighDc = 0;
int lowBound = 0;
int highBound = 0;
int bestHighDc = 0;
float minCurrent = 0.0f;
float maxCurrent = 0.0f;
float minCurrentDifference = 0.0f;

// Measurement variables
int measureStep = 0;
std::vector<float> voltagesLoaded;
std::vector<float> currentsLoaded;
std::vector<float> ir_dutyCycles;
std::vector<float> consecutiveInternalResistances;

// Results storage
float internalResistanceData[MAX_RESISTANCE_POINTS][2];
int resistanceDataCount = 0;
float internalResistanceDataPairs[MAX_RESISTANCE_POINTS][2];
int resistanceDataCountPairs = 0;

// Regression results
float regressedInternalResistanceSlope = 0.0f;
float regressedInternalResistanceIntercept = 0.0f;
float regressedInternalResistancePairsSlope = 0.0f;
float regressedInternalResistancePairsIntercept = 0.0f;

bool isMeasuringResistance = false;

ElectrodeParameters g_electrode;

// Minimum useful number of samples.
static constexpr size_t MIN_ELECTRODE_SAMPLES = 12;

// The first samples represent the immediate post-step region.
static constexpr size_t INITIAL_SAMPLE_COUNT = 4;

// Number of samples used to characterize settled current.
static constexpr size_t FINAL_SAMPLE_COUNT = 8;

// Current stability requirement in the final region (0.02 = 2%).
static constexpr float MAX_FINAL_CURRENT_RELATIVE_SPREAD = 0.02f;

// Minimum acceptable coefficient of determination.
static constexpr float MIN_FIT_R2 = 0.95f;

// Require enough measurement duration to observe most of the RC transient.
static constexpr float MIN_TIME_SPAN_TO_TAU = 4.0f;

// Search limits for tau.
static constexpr float MIN_TAU_S = 0.001f;
static constexpr float MAX_TAU_S = 100.0f;

// Physical sanity limits.
static constexpr float MIN_RESISTANCE_OHM = 1.0e-6f;
static constexpr float MAX_RESISTANCE_OHM = 1.0e6f;

static constexpr float MIN_CAPACITANCE_F = 1.0e-9f;
static constexpr float MAX_CAPACITANCE_F = 1.0e9f;

// Use 3.5 tau for ~97% settling.
static constexpr float SETTLING_TAU_MULTIPLIER = 3.5f;

// Maximum delay allowed for the caller.
static constexpr uint32_t MAX_ADAPTIVE_DELAY_MS = 600000UL;

// Number of points in the initial logarithmic tau search.
static constexpr int TAU_GRID_POINTS = 80;

static bool finiteFloat(float x)
{
    return std::isfinite(x);
}

static bool rangeMean(
    const float *values,
    size_t begin,
    size_t end,
    float &result)
{
    if (!values || end <= begin)
        return false;

    double sum = 0.0;

    for (size_t i = begin; i < end; ++i)
    {
        if (!finiteFloat(values[i]))
            return false;

        sum += values[i];
    }

    result = (float)(sum / (double)(end - begin));
    return finiteFloat(result);
}

static bool checkFinalCurrentStability(
    const float *current,
    size_t count,
    float &finalCurrent)
{
    if (!current || count < FINAL_SAMPLE_COUNT)
        return false;

    const size_t begin = count - FINAL_SAMPLE_COUNT;

    float mean = 0.0f;

    if (!rangeMean(current, begin, count, mean))
        return false;

    float minI = current[begin];
    float maxI = current[begin];

    for (size_t i = begin; i < count; ++i)
    {
        if (!finiteFloat(current[i]))
            return false;

        if (current[i] < minI)
            minI = current[i];

        if (current[i] > maxI)
            maxI = current[i];
    }

    const float denominator = fmaxf(fabsf(mean), 1.0e-6f);
    const float relativeSpread =
        (maxI - minI) / denominator;

    finalCurrent = mean;

    return relativeSpread <= MAX_FINAL_CURRENT_RELATIVE_SPREAD;
}

static bool fitForTau(
    const float *t,
    const float *v,
    size_t n,
    float tau,
    float &c,
    float &A,
    float &sse)
{
    if (!t || !v || n < MIN_ELECTRODE_SAMPLES)
        return false;

    if (!finiteFloat(tau) || tau <= 0.0f)
        return false;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        const float dt = t[i] - t[0];

        if (!finiteFloat(dt) || !finiteFloat(v[i]) || dt < 0.0f)
            return false;

        const float exponent = -dt / tau;

        float x;

        if (exponent < -80.0f)
            x = 0.0f;
        else
            x = expf(exponent);

        if (!finiteFloat(x))
            return false;

        const double xd = x;
        const double yd = v[i];

        sumX += xd;
        sumY += yd;
        sumXX += xd * xd;
        sumXY += xd * yd;
    }

    const double nd = (double)n;

    const double denominator =
        nd * sumXX - sumX * sumX;

    if (fabs(denominator) < 1.0e-15)
        return false;

    const double Ad =
        (nd * sumXY - sumX * sumY) / denominator;

    const double cd =
        (sumY - Ad * sumX) / nd;

    if (!std::isfinite(Ad) || !std::isfinite(cd))
        return false;

    double error = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        const float dt = t[i] - t[0];
        const float exponent = -dt / tau;

        float x =
            (exponent < -80.0f)
            ? 0.0f
            : expf(exponent);

        const double predicted =
            cd + Ad * (double)x;

        const double residual =
            (double)v[i] - predicted;

        error += residual * residual;
    }

    if (!std::isfinite(error))
        return false;

    c = (float)cd;
    A = (float)Ad;
    sse = (float)error;

    return finiteFloat(c) &&
           finiteFloat(A) &&
           finiteFloat(sse);
}

static bool findBestTau(
    const float *t,
    const float *v,
    size_t n,
    float minTau,
    float maxTau,
    float &bestTau,
    float &bestSSE)
{
    if (minTau <= 0.0f ||
        maxTau <= minTau)
        return false;

    const float logMin = logf(minTau);
    const float logMax = logf(maxTau);

    bestSSE = INFINITY;
    bestTau = NAN;

    for (int k = 0; k < TAU_GRID_POINTS; ++k)
    {
        const float fraction =
            (float)k / (float)(TAU_GRID_POINTS - 1);

        const float logTau =
            logMin + fraction * (logMax - logMin);

        const float tau = expf(logTau);

        float c, A, sse;

        if (!fitForTau(
                t,
                v,
                n,
                tau,
                c,
                A,
                sse))
        {
            continue;
        }

        if (sse < bestSSE)
        {
            bestSSE = sse;
            bestTau = tau;
        }
    }

    return finiteFloat(bestTau) &&
           finiteFloat(bestSSE);
}

static bool refineTau(
    const float *t,
    const float *v,
    size_t n,
    float lowerTau,
    float upperTau,
    float &bestTau,
    float &bestSSE)
{
    if (lowerTau <= 0.0f ||
        upperTau <= lowerTau)
        return false;

    float a = logf(lowerTau);
    float b = logf(upperTau);

    const float phi =
        0.6180339887498948482f;

    float c = b - phi * (b - a);
    float d = a + phi * (b - a);

    auto evaluateLogTau =
        [&](float logTau, float &sse) -> bool
    {
        const float tau = expf(logTau);

        float fitC, fitA;

        return fitForTau(
            t,
            v,
            n,
            tau,
            fitC,
            fitA,
            sse);
    };

    float fc, fd;

    if (!evaluateLogTau(c, fc) ||
        !evaluateLogTau(d, fd))
    {
        return false;
    }

    for (int iteration = 0;
         iteration < 48;
         ++iteration)
    {
        if (fc < fd)
        {
            b = d;
            d = c;
            fd = fc;

            c = b - phi * (b - a);

            if (!evaluateLogTau(c, fc))
                return false;
        }
        else
        {
            a = c;
            c = d;
            fc = fd;

            d = a + phi * (b - a);

            if (!evaluateLogTau(d, fd))
                return false;
        }
    }

    const float logTau =
        0.5f * (a + b);

    const float tau =
        expf(logTau);

    float finalC, finalA, finalSSE;

    if (!fitForTau(
            t,
            v,
            n,
            tau,
            finalC,
            finalA,
            finalSSE))
    {
        return false;
    }

    bestTau = tau;
    bestSSE = finalSSE;

    return finiteFloat(bestTau) &&
           finiteFloat(bestSSE);
}

static bool calculateFitQuality(
    const float *t,
    const float *v,
    size_t n,
    float tau,
    float &c,
    float &A,
    float &r2,
    float &rmse)
{
    float sse;

    if (!fitForTau(
            t,
            v,
            n,
            tau,
            c,
            A,
            sse))
    {
        return false;
    }

    double meanV = 0.0;

    for (size_t i = 0; i < n; ++i)
        meanV += v[i];

    meanV /= (double)n;

    double sst = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        const double d =
            (double)v[i] - meanV;

        sst += d * d;
    }

    if (sst <= 1.0e-20)
        return false;

    r2 =
        1.0f - sse / (float)sst;

    rmse =
        sqrtf(sse / (float)n);

    return finiteFloat(r2) &&
           finiteFloat(rmse);
}

bool evaluateElectrodeParameters(
    const ElectrodeTransient &measurement)
{
    ElectrodeParameters result;

    if (!measurement.time_s ||
        !measurement.voltage_V ||
        !measurement.current_A)
    {
        Serial.println(
            "Electrode characterization: null data pointer.");
        return false;
    }

    if (measurement.count < MIN_ELECTRODE_SAMPLES)
    {
        Serial.println(
            "Electrode characterization: insufficient samples.");
        return false;
    }

    if (!finiteFloat(measurement.v_unloaded) ||
        !finiteFloat(measurement.i_before_A))
    {
        Serial.println(
            "Electrode characterization: invalid baseline.");
        return false;
    }

    for (size_t i = 0;
         i < measurement.count;
         ++i)
    {
        if (!finiteFloat(measurement.time_s[i]) ||
            !finiteFloat(measurement.voltage_V[i]) ||
            !finiteFloat(measurement.current_A[i]))
        {
            Serial.printf(
                "Electrode characterization: invalid sample %u.\n",
                (unsigned)i);

            return false;
        }

        if (i > 0 &&
            measurement.time_s[i] <=
            measurement.time_s[i - 1])
        {
            Serial.println(
                "Electrode characterization: timestamps are not strictly increasing.");
            return false;
        }
    }

    const float measurementDuration =
        measurement.time_s[measurement.count - 1] -
        measurement.time_s[0];

    if (!finiteFloat(measurementDuration) ||
        measurementDuration <= 0.0f)
    {
        Serial.println(
            "Electrode characterization: invalid time span.");
        return false;
    }

    float finalCurrent = NAN;

    const bool currentStable =
        checkFinalCurrentStability(
            measurement.current_A,
            measurement.count,
            finalCurrent);

    result.currentStable = currentStable;

    if (!currentStable)
    {
        Serial.println(
            "Electrode characterization: final current is not stable.");
        return false;
    }

    const float deltaI =
        finalCurrent -
        measurement.i_before_A;

    result.deltaI_A = deltaI;

    if (!finiteFloat(deltaI) ||
        fabsf(deltaI) < MEASURABLE_CURRENT_THRESHOLD)
    {
        Serial.printf(
            "Electrode characterization: current step too small: %.6f A\n",
            deltaI);

        return false;
    }

    const size_t initialCount =
        (measurement.count < INITIAL_SAMPLE_COUNT)
        ? measurement.count
        : INITIAL_SAMPLE_COUNT;

    float V0Measured = NAN;

    if (!rangeMean(
            measurement.voltage_V,
            0,
            initialCount,
            V0Measured))
    {
        Serial.println(
            "Electrode characterization: failed initial voltage calculation.");
        return false;
    }

    const float maximumIdentifiableTau =
        measurementDuration / MIN_TIME_SPAN_TO_TAU;

    result.sufficientTime =
        maximumIdentifiableTau >= MIN_TAU_S;

    if (!result.sufficientTime)
    {
        Serial.printf(
            "Electrode characterization: measurement too short: %.6f s\n",
            measurementDuration);

        return false;
    }

    const float tauUpper =
        fminf(
            MAX_TAU_S,
            maximumIdentifiableTau);

    const float tauLower =
        MIN_TAU_S;

    if (tauUpper <= tauLower)
    {
        Serial.println(
            "Electrode characterization: invalid tau search interval.");
        return false;
    }

    float coarseTau = NAN;
    float coarseSSE = NAN;

    if (!findBestTau(
            measurement.time_s,
            measurement.voltage_V,
            measurement.count,
            tauLower,
            tauUpper,
            coarseTau,
            coarseSSE))
    {
        Serial.println(
            "Electrode characterization: tau search failed.");
        return false;
    }

    const float gridRatio =
        expf(
            (logf(tauUpper) - logf(tauLower)) /
            (float)(TAU_GRID_POINTS - 1));

    float refineLower =
        coarseTau / gridRatio;

    float refineUpper =
        coarseTau * gridRatio;

    refineLower =
        fmaxf(tauLower, refineLower);

    refineUpper =
        fminf(tauUpper, refineUpper);

    float fittedTau = NAN;
    float fittedSSE = NAN;

    if (!refineTau(
            measurement.time_s,
            measurement.voltage_V,
            measurement.count,
            refineLower,
            refineUpper,
            fittedTau,
            fittedSSE))
    {
        fittedTau = coarseTau;
        fittedSSE = coarseSSE;
    }

    float VInfinity = NAN;
    float amplitude = NAN;
    float fitR2 = NAN;
    float fitRMSE = NAN;

    if (!calculateFitQuality(
            measurement.time_s,
            measurement.voltage_V,
            measurement.count,
            fittedTau,
            VInfinity,
            amplitude,
            fitR2,
            fitRMSE))
    {
        Serial.println(
            "Electrode characterization: final fit quality calculation failed.");
        return false;
    }

    result.fitR2 = fitR2;
    result.fitRMSE_V = fitRMSE;
    result.fittedVInfinity = VInfinity;
    result.fittedAmplitude = amplitude;
    result.tau_rc = fittedTau;

    if (fitR2 < MIN_FIT_R2)
    {
        Serial.printf(
            "Electrode characterization: poor RC fit, R2=%.5f\n",
            fitR2);

        return false;
    }

    result.fitValid = true;

    const float fittedV0 =
        VInfinity + amplitude;

    if (!finiteFloat(fittedV0))
    {
        Serial.println(
            "Electrode characterization: invalid fitted initial voltage.");
        return false;
    }

    result.fittedV0 = fittedV0;

    const float deltaVOhmic =
        fittedV0 -
        measurement.v_unloaded;

    const float RohmicSigned =
        deltaVOhmic /
        deltaI;

    const float Rohmic =
        fabsf(RohmicSigned);

    if (!finiteFloat(Rohmic) ||
        Rohmic < MIN_RESISTANCE_OHM ||
        Rohmic > MAX_RESISTANCE_OHM)
    {
        Serial.printf(
            "Electrode characterization: invalid Rohmic=%.9f Ohm\n",
            Rohmic);

        return false;
    }

    result.R_ohmic = Rohmic;

    const float Rct =
        fabsf(amplitude / deltaI);

    if (!finiteFloat(Rct) ||
        Rct < MIN_RESISTANCE_OHM ||
        Rct > MAX_RESISTANCE_OHM)
    {
        Serial.printf(
            "Electrode characterization: invalid Rct=%.9f Ohm\n",
            Rct);

        return false;
    }

    result.R_ct = Rct;

    const float Cdl =
        fittedTau / Rct;

    if (!finiteFloat(Cdl) ||
        Cdl < MIN_CAPACITANCE_F ||
        Cdl > MAX_CAPACITANCE_F)
    {
        Serial.printf(
            "Electrode characterization: invalid Cdl=%.9e F\n",
            Cdl);

        return false;
    }

    result.C_dl = Cdl;

    if (measurement.specificCdl_F_per_m2 > 0.0f &&
        finiteFloat(measurement.specificCdl_F_per_m2))
    {
        const float area =
            Cdl /
            measurement.specificCdl_F_per_m2;

        if (finiteFloat(area) &&
            area > 0.0f &&
            area < 1.0e9f)
        {
            result.activeSurfaceAreaM2 = area;
        }

        result.activeSurfaceAreaProxy =
            result.activeSurfaceAreaM2;
    }
    else
    {
        result.activeSurfaceAreaM2 = NAN;
        result.activeSurfaceAreaProxy = Cdl;
    }

    const float requiredDelayS =
        SETTLING_TAU_MULTIPLIER *
        fittedTau;

    if (!finiteFloat(requiredDelayS) ||
        requiredDelayS <= 0.0f)
    {
        Serial.println(
            "Electrode characterization: invalid settling delay.");
        return false;
    }

    const double delayMsDouble =
        (double)requiredDelayS *
        1000.0;

    result.delayLimited =
        delayMsDouble >
        (double)MAX_ADAPTIVE_DELAY_MS;

    if (result.delayLimited)
    {
        result.adaptiveDelayMs =
            MAX_ADAPTIVE_DELAY_MS;
    }
    else
    {
        result.adaptiveDelayMs =
            (uint32_t)ceil(delayMsDouble);
    }

    bool physicalOK = true;

    const float totalPolarization =
        fabsf(amplitude);

    if (!finiteFloat(totalPolarization))
        physicalOK = false;

    if (totalPolarization < 1.0e-6f)
        physicalOK = false;

    const float signalSpan =
        fmaxf(
            fabsf(V0Measured - VInfinity),
            1.0e-6f);

    if (fitRMSE > signalSpan * 0.25f)
        physicalOK = false;

    if (!physicalOK)
    {
        Serial.println(
            "Electrode characterization: physical consistency check failed.");
        return false;
    }

    result.physicallyValid = true;

    WEB_LOCK();

    g_electrode = result;
    g_electrode.evaluated = true;

    WEB_UNLOCK();

    Serial.println("");
    Serial.println("========== ELECTRODE CHARACTERIZATION ==========");

    Serial.printf(
        "Samples              : %u\n",
        (unsigned)measurement.count);

    Serial.printf(
        "Measurement span     : %.6f s\n",
        measurementDuration);

    Serial.printf(
        "Initial voltage      : %.6f V\n",
        measurement.v_unloaded);

    Serial.printf(
        "Fitted V(0+)         : %.6f V\n",
        fittedV0);

    Serial.printf(
        "Fitted V(infinity)   : %.6f V\n",
        VInfinity);

    Serial.printf(
        "Fitted amplitude     : %.6f V\n",
        amplitude);

    Serial.printf(
        "Current before       : %.6f A\n",
        measurement.i_before_A);

    Serial.printf(
        "Current final        : %.6f A\n",
        finalCurrent);

    Serial.printf(
        "Delta I              : %.6f A\n",
        deltaI);

    Serial.printf(
        "R_ohmic              : %.9f Ohm\n",
        result.R_ohmic);

    Serial.printf(
        "R_ct / R_pol         : %.9f Ohm\n",
        result.R_ct);

    Serial.printf(
        "Tau                  : %.6f s\n",
        result.tau_rc);

    Serial.printf(
        "C_dl                 : %.9e F\n",
        result.C_dl);

    if (finiteFloat(result.activeSurfaceAreaM2))
    {
        Serial.printf(
            "Active area          : %.9e m^2\n",
            result.activeSurfaceAreaM2);
    }
    else
    {
        Serial.println(
            "Active area          : not calibrated");
    }

    Serial.printf(
        "Fit R2               : %.6f\n",
        result.fitR2);

    Serial.printf(
        "Fit RMSE             : %.9f V\n",
        result.fitRMSE_V);

    Serial.printf(
        "Adaptive delay       : %lu ms%s\n",
        (unsigned long)result.adaptiveDelayMs,
        result.delayLimited ? " (LIMITED)" : "");

    Serial.printf(
        "Current stable       : %s\n",
        result.currentStable ? "YES" : "NO");

    Serial.printf(
        "RC fit valid         : %s\n",
        result.fitValid ? "YES" : "NO");

    Serial.printf(
        "Physical validity    : %s\n",
        result.physicallyValid ? "YES" : "NO");

    Serial.println(
        "================================================");
    Serial.println("");

    return true;
}

bool evaluateElectrodeParameters(
    const float *time_s,
    const float *voltage_V,
    const float *current_A,
    size_t count,
    float v_unloaded,
    float i_before_A,
    float specificCdl_F_per_m2)
{
    ElectrodeTransient measurement;

    measurement.time_s = time_s;
    measurement.voltage_V = voltage_V;
    measurement.current_A = current_A;
    measurement.count = count;
    measurement.v_unloaded = v_unloaded;
    measurement.i_before_A = i_before_A;
    measurement.specificCdl_F_per_m2 =
        specificCdl_F_per_m2;

    return evaluateElectrodeParameters(measurement);
}

// Helper function to initiate measurement
void getSingleMeasurement(int dc, IRState nextState) {
    applyDuty(dc);
    irStateChangeTime = millis();
    nextIRState = nextState;
    currentIRState = IR_STATE_GET_MEASUREMENT;
}

// Reset all state variables for new measurement
void resetMeasurementState() {
    WEB_LOCK();
    resistanceDataCount = 0;
    resistanceDataCountPairs = 0;
    WEB_UNLOCK();

    voltagesLoaded.clear();
    voltagesLoaded.reserve(MAX_RESISTANCE_POINTS);

    currentsLoaded.clear();
    currentsLoaded.reserve(MAX_RESISTANCE_POINTS);

    ir_dutyCycles.clear();
    ir_dutyCycles.reserve(MAX_RESISTANCE_POINTS);

    consecutiveInternalResistances.clear();
    consecutiveInternalResistances.reserve(MAX_RESISTANCE_POINTS);

    dutyCyclePairs.clear();
    dutyCyclePairs.reserve(MAX_RESISTANCE_POINTS / 2);

    pairIndex = 0;
    measureStep = 0;
    pairGenerationStep = 0;
    pairGenerationSubStep = 0;

    findMinDcLow = 0;
    findMinDcHigh = 0;
    findMinDcMid = 0;
    minimalDutyCycle = 0;

    minCurrent = 0.0f;
    maxCurrent = 0.0f;
    lowDc = 0;
    previousHighDc = 0;
    bestHighDc = 0;
    minCurrentDifference = 0.0f;
}

void measureInternalResistanceStep() {
    unsigned long now = millis();

    switch (currentIRState) {
        case IR_STATE_IDLE:
            break;

        case IR_STATE_START:
            Serial.println("Starting improved internal resistance measurement...");
            resetMeasurementState();
            currentIRState = IR_STATE_STOP_LOAD_WAIT;
            irStateChangeTime = now;
            applyDuty(0);
            break;

        case IR_STATE_STOP_LOAD_WAIT:
            if (now - irStateChangeTime >= UNLOADED_VOLTAGE_DELAY_MS) {
                currentIRState = IR_STATE_GET_UNLOADED_VOLTAGE;
            }
            break;

        case IR_STATE_GET_UNLOADED_VOLTAGE:
            {
                MeasurementData initialUnloaded;
                getThermistorReadings(initialUnloaded.temp1, initialUnloaded.temp2,
                                     initialUnloaded.tempDiff, initialUnloaded.t1_millivolts,
                                     initialUnloaded.voltage, initialUnloaded.current);
                Serial.printf("Initial Unloaded Voltage: %.3f V\n", initialUnloaded.voltage);

                minimalDutyCycle = estimateDutyCycleForCurrent(MEASURABLE_CURRENT_THRESHOLD);
                if (minimalDutyCycle < MIN_DUTY_CYCLE_START) minimalDutyCycle = MIN_DUTY_CYCLE_START;

                Serial.printf("Minimal duty cycle (from model): %d\n", minimalDutyCycle);
                currentIRState = IR_STATE_GENERATE_PAIRS;
            }
            break;

        case IR_STATE_FIND_MIN_DC:
            // This state is now skipped but kept for state machine integrity if needed
            currentIRState = IR_STATE_GENERATE_PAIRS;
            break;

        case IR_STATE_GENERATE_PAIRS:
            handleGeneratePairs();
            break;

        case IR_STATE_MEASURE_L_UL:
            handleMeasureLoadedUnloaded();
            break;

        case IR_STATE_MEASURE_PAIRS:
            handleMeasurePairs();
            break;

        case IR_STATE_GET_MEASUREMENT:
            {
                unsigned long reqDelay = g_electrode.evaluated ? g_electrode.adaptiveDelayMs : STABILIZATION_DELAY_MS;
                if (now - irStateChangeTime >= reqDelay) {
                    getThermistorReadings(currentMeasurement.temp1, currentMeasurement.temp2,
                                         currentMeasurement.tempDiff, currentMeasurement.t1_millivolts,
                                         currentMeasurement.voltage, currentMeasurement.current);
                    currentMeasurement.dutyCycle = (uint8_t)dutyCycle;
                    currentMeasurement.timestamp = (uint32_t)millis();
                    currentIRState = nextIRState;
                }
            }
            break;

        case IR_STATE_COMPLETE:
            completeResistanceMeasurement();
            break;
    }
}

void handleGeneratePairs() {
    switch (pairGenerationStep) {
        case 0:
            if (minimalDutyCycle == 0) {
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            getSingleMeasurement(minimalDutyCycle, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 1;
            break;

        case 1:
            minCurrent = currentMeasurement.current;
            getSingleMeasurement(MAX_DUTY_CYCLE, IR_STATE_GENERATE_PAIRS);
            pairGenerationStep = 2;
            break;

        case 2:
            maxCurrent = currentMeasurement.current;
            if (maxCurrent <= minCurrent) {
                currentIRState = IR_STATE_IDLE;
                isMeasuringResistance = false;
                return;
            }
            lowDc = minimalDutyCycle;
            previousHighDc = MAX_DUTY_CYCLE;
            pairIndex = 0;
            pairGenerationStep = 3;
            pairGenerationSubStep = 0;
            // Fall through

        case 3:
            handlePairGeneration();
            break;
    }
}

bool isDutyCycleLinearRegion(int dc, float& out_slope) {
    out_slope = 0.0f;
    if (dc < MIN_DUTY_CYCLE_START || dc > MAX_DUTY_CYCLE) return false;
    float currentEst = estimateCurrent(dc);
    if (currentEst < MEASURABLE_CURRENT_THRESHOLD) return false;

    int lowDC = std::max(MIN_CHARGE_DUTY_CYCLE, dc - 1);
    int highDC = std::min(MAX_DUTY_CYCLE, dc + 1);
    float lowI = estimateCurrent(lowDC);
    float highI = estimateCurrent(highDC);
    out_slope = (highI - lowI) / static_cast<float>(highDC - lowDC);

    if (out_slope < MIN_VALID_DUTY_MODEL_SLOPE) return false;
    return true;
}

bool isCurrentCovered(float current, float tolerance) {
    WEB_LOCK();
    int countPairs = std::clamp(resistanceDataCountPairs, 0, (int)MAX_RESISTANCE_POINTS);
    for (int i = 0; i < countPairs; ++i) {
        if (std::fabs(internalResistanceDataPairs[i][0] - current) < tolerance) {
            WEB_UNLOCK();
            return true;
        }
    }
    int countLu = std::clamp(resistanceDataCount, 0, (int)MAX_RESISTANCE_POINTS);
    for (int i = 0; i < countLu; ++i) {
        if (std::fabs(internalResistanceData[i][0] - current) < tolerance) {
            WEB_UNLOCK();
            return true;
        }
    }
    WEB_UNLOCK();
    return false;
}

void computeIntervalMidpointsAndBlindSpots(std::vector<CandidatePoint>& candidates, float minI, float maxI, float activeCurrent) {
    candidates.clear();
    std::vector<float> uniqueCurrents;

    WEB_LOCK();
    int countPairs = std::clamp(resistanceDataCountPairs, 0, (int)MAX_RESISTANCE_POINTS);
    uniqueCurrents.reserve(countPairs + std::clamp(resistanceDataCount, 0, (int)MAX_RESISTANCE_POINTS) + 2);
    for (int i = 0; i < countPairs; ++i) {
        float cur = internalResistanceDataPairs[i][0];
        if (cur >= minI && cur <= maxI) uniqueCurrents.push_back(cur);
    }
    int countLu = std::clamp(resistanceDataCount, 0, (int)MAX_RESISTANCE_POINTS);
    for (int i = 0; i < countLu; ++i) {
        float cur = internalResistanceData[i][0];
        if (cur >= minI && cur <= maxI) uniqueCurrents.push_back(cur);
    }
    WEB_UNLOCK();

    uniqueCurrents.push_back(minI);
    uniqueCurrents.push_back(maxI);

    std::sort(uniqueCurrents.begin(), uniqueCurrents.end());
    uniqueCurrents.erase(std::unique(uniqueCurrents.begin(), uniqueCurrents.end(),
        [](float a, float b) { return std::fabs(a - b) < 0.003f; }), uniqueCurrents.end());

    if (uniqueCurrents.size() < 2) {
        float mid = (minI + maxI) / 2.0f;
        CandidatePoint cp;
        cp.current = mid;
        cp.duty = estimateDutyCycleForCurrent(mid);
        cp.score = 1.0f;
        cp.type = PAIR_TYPE_RANDOM_BLINDSPOT;
        candidates.push_back(cp);
        return;
    }

    for (size_t k = 0; k < uniqueCurrents.size() - 1; ++k) {
        float i1 = uniqueCurrents[k];
        float i2 = uniqueCurrents[k + 1];
        float gapWidth = i2 - i1;
        float midI = (i1 + i2) / 2.0f;

        float dummySlope = 0.0f;
        int dcMid = estimateDutyCycleForCurrent(midI);
        if (!isDutyCycleLinearRegion(dcMid, dummySlope)) continue;

        if (!isCurrentCovered(midI, EXPLORATION_TOLERANCE_CURRENT)) {
            float distToActive = std::fabs(midI - activeCurrent);
            float relevance = 1.0f / (1.0f + 5.0f * distToActive);
            float score = gapWidth * relevance;

            CandidatePoint cp;
            cp.current = midI;
            cp.duty = dcMid;
            cp.score = score;
            cp.type = PAIR_TYPE_RANDOM_BLINDSPOT;
            candidates.push_back(cp);
        }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const CandidatePoint& a, const CandidatePoint& b) { return a.score > b.score; });
}

void findCurrentBlindSpots(float gaps[][2], int& gapCount, float maxOperatingCurrent) {
    gapCount = 0;
    std::vector<CandidatePoint> candidates;
    computeIntervalMidpointsAndBlindSpots(candidates, MEASURABLE_CURRENT_THRESHOLD, maxOperatingCurrent, maximumCurrent);
    for (const auto& cp : candidates) {
        if (gapCount >= 5) break;
        gaps[gapCount][0] = cp.current - 0.01f;
        gaps[gapCount][1] = cp.current + 0.01f;
        gapCount++;
    }
}

void allocateReevaluationCandidates(std::vector<RePoint>& points, int budget) {
    points.clear();
    if (budget <= 0) return;

    extern int minimalDutyCycle;
    int minDC = minimalDutyCycle;
    if (minDC < MIN_DUTY_CYCLE_START) minDC = MIN_DUTY_CYCLE_START;
    int maxDC = MAX_DUTY_CYCLE;

    float slopeMin = 0.0f, slopeMax = 0.0f;
    while (minDC < maxDC && !isDutyCycleLinearRegion(minDC, slopeMin)) minDC++;
    while (maxDC > minDC && !isDutyCycleLinearRegion(maxDC, slopeMax)) maxDC--;

    float minI = estimateCurrent(minDC);
    float maxI = estimateCurrent(maxDC);
    float activeI = maximumCurrent;

    int verificationQuota = std::max(1, (int)(0.25f * budget));

    // 1. Exploitation / High-Value Verification Anchors
    int dcActive = estimateDutyCycleForCurrent(activeI);
    if (dcActive >= minDC && dcActive <= maxDC) {
        RePoint pAct;
        pAct.current = activeI;
        pAct.duty = dcActive;
        pAct.isPair = true;
        points.push_back(pAct);
    }

    if ((int)points.size() < verificationQuota && (maxI - minI >= GLOBAL_PAIR_MIN_DELTA_I)) {
        if (!isCurrentCovered(minI, EXPLORATION_TOLERANCE_CURRENT * 2.0f)) {
            RePoint pMin; pMin.current = minI; pMin.duty = minDC; pMin.isPair = true;
            points.push_back(pMin);
        }
        if ((int)points.size() < verificationQuota && !isCurrentCovered(maxI, EXPLORATION_TOLERANCE_CURRENT * 2.0f)) {
            RePoint pMax; pMax.current = maxI; pMax.duty = maxDC; pMax.isPair = true;
            points.push_back(pMax);
        }
    }

    // 2. Exploration / Top-Scoring Interval Midpoints
    std::vector<CandidatePoint> candidates;
    computeIntervalMidpointsAndBlindSpots(candidates, minI, maxI, activeI);

    for (const auto& cand : candidates) {
        if ((int)points.size() >= budget) break;
        if (!isCurrentCovered(cand.current, EXPLORATION_TOLERANCE_CURRENT)) {
            RePoint pExpl;
            pExpl.current = cand.current;
            pExpl.duty = cand.duty;
            pExpl.isPair = true;
            points.push_back(pExpl);
        }
    }

    // 3. Fallback: bisect intervals if candidates exhausted
    while ((int)points.size() < budget) {
        float randI = minI + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * std::max(0.01f, maxI - minI);
        int dcRand = estimateDutyCycleForCurrent(randI);
        if (dcRand >= minDC && dcRand <= maxDC && !isCurrentCovered(randI, EXPLORATION_TOLERANCE_CURRENT)) {
            RePoint pRand;
            pRand.current = randI;
            pRand.duty = dcRand;
            pRand.isPair = false;
            points.push_back(pRand);
        } else {
            break;
        }
    }
}

void generateCategorizedDutyPairs(std::vector<DutyPair>& pairs, int maxPairs) {
    pairs.clear();
    int minDC = minimalDutyCycle;
    if (minDC < MIN_DUTY_CYCLE_START) minDC = MIN_DUTY_CYCLE_START;
    int maxDC = MAX_DUTY_CYCLE;

    float slopeMin = 0.0f, slopeMax = 0.0f;
    while (minDC < maxDC && !isDutyCycleLinearRegion(minDC, slopeMin)) minDC++;
    while (maxDC > minDC && !isDutyCycleLinearRegion(maxDC, slopeMax)) maxDC--;

    float minI = estimateCurrent(minDC);
    float maxI = estimateCurrent(maxDC);
    float spanI = maxI - minI;

    if (spanI < 0.05f) {
        DutyPair p;
        p.lowDC = minDC;
        p.highDC = maxDC;
        p.type = PAIR_TYPE_GLOBAL;
        p.targetLowCurrent = minI;
        p.targetHighCurrent = maxI;
        pairs.push_back(p);
        return;
    }

    // 1. Global Pairs (High Quality, Large Delta I, stable baseline/mean)
    DutyPair g1;
    g1.lowDC = minDC;
    g1.highDC = maxDC;
    g1.type = PAIR_TYPE_GLOBAL;
    g1.targetLowCurrent = minI;
    g1.targetHighCurrent = maxI;
    pairs.push_back(g1);

    if (maxPairs > 2) {
        int g2_low = estimateDutyCycleForCurrent(minI + 0.15f * spanI);
        int g2_high = estimateDutyCycleForCurrent(minI + 0.85f * spanI);
        if (g2_high - g2_low > 10) {
            DutyPair g2;
            g2.lowDC = std::max(minDC, g2_low);
            g2.highDC = std::min(maxDC, g2_high);
            g2.type = PAIR_TYPE_GLOBAL;
            g2.targetLowCurrent = estimateCurrent(g2.lowDC);
            g2.targetHighCurrent = estimateCurrent(g2.highDC);
            pairs.push_back(g2);
        }
    }

    // 2. Local Pairs (Close currents, local accuracy & derivative)
    float localStepI = std::min(LOCAL_PAIR_MAX_DELTA_I, std::max(LOCAL_PAIR_MIN_DELTA_I, spanI * 0.15f));

    float l_low1 = minI + 0.05f * spanI;
    int dc_l_low1 = estimateDutyCycleForCurrent(l_low1);
    int dc_l_low2 = estimateDutyCycleForCurrent(l_low1 + localStepI);
    if (dc_l_low2 > dc_l_low1) {
        DutyPair l1;
        l1.lowDC = dc_l_low1; l1.highDC = dc_l_low2;
        l1.type = PAIR_TYPE_LOCAL;
        l1.targetLowCurrent = estimateCurrent(dc_l_low1);
        l1.targetHighCurrent = estimateCurrent(dc_l_low2);
        pairs.push_back(l1);
    }

    float l_mid1 = minI + 0.45f * spanI;
    int dc_l_mid1 = estimateDutyCycleForCurrent(l_mid1);
    int dc_l_mid2 = estimateDutyCycleForCurrent(l_mid1 + localStepI);
    if (dc_l_mid2 > dc_l_mid1 && (int)pairs.size() < maxPairs) {
        DutyPair l2;
        l2.lowDC = dc_l_mid1; l2.highDC = dc_l_mid2;
        l2.type = PAIR_TYPE_LOCAL;
        l2.targetLowCurrent = estimateCurrent(dc_l_mid1);
        l2.targetHighCurrent = estimateCurrent(dc_l_mid2);
        pairs.push_back(l2);
    }

    // High current pair: Anchor against minDC to ensure large Delta I and prevent high-end saturation divergence
    float l_high1 = minI + 0.85f * spanI;
    int dc_l_high2 = estimateDutyCycleForCurrent(l_high1);
    int dc_l_high1 = minDC;
    if (dc_l_high2 > dc_l_high1 && (int)pairs.size() < maxPairs) {
        DutyPair l3;
        l3.lowDC = dc_l_high1; l3.highDC = dc_l_high2;
        l3.type = PAIR_TYPE_LOCAL;
        l3.targetLowCurrent = estimateCurrent(dc_l_high1);
        l3.targetHighCurrent = estimateCurrent(dc_l_high2);
        pairs.push_back(l3);
    }

    // 3. Random / Blind-Spot Pairs
    float blindSpots[5][2];
    int gapCount = 0;
    findCurrentBlindSpots(blindSpots, gapCount, maxI);
    for (int k = 0; k < gapCount && (int)pairs.size() < maxPairs; k++) {
        float gapMid = (blindSpots[k][0] + blindSpots[k][1]) / 2.0f;
        int dc1 = estimateDutyCycleForCurrent(gapMid - localStepI * 0.5f);
        int dc2 = estimateDutyCycleForCurrent(gapMid + localStepI * 0.5f);
        if (dc2 > dc1) {
            DutyPair r1;
            r1.lowDC = std::max(minDC, dc1);
            r1.highDC = std::min(maxDC, dc2);
            r1.type = PAIR_TYPE_RANDOM_BLINDSPOT;
            r1.targetLowCurrent = estimateCurrent(r1.lowDC);
            r1.targetHighCurrent = estimateCurrent(r1.highDC);
            pairs.push_back(r1);
        }
    }

    while ((int)pairs.size() < maxPairs) {
        float randFraction = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float rI1 = minI + randFraction * std::max(0.01f, maxI - minI - localStepI);
        int rDC1 = estimateDutyCycleForCurrent(rI1);
        int rDC2 = estimateDutyCycleForCurrent(rI1 + localStepI);
        if (rDC2 > rDC1) {
            DutyPair rPair;
            rPair.lowDC = std::max(minDC, rDC1);
            rPair.highDC = std::min(maxDC, rDC2);
            rPair.type = PAIR_TYPE_RANDOM_BLINDSPOT;
            rPair.targetLowCurrent = estimateCurrent(rPair.lowDC);
            rPair.targetHighCurrent = estimateCurrent(rPair.highDC);
            pairs.push_back(rPair);
        } else {
            break;
        }
    }
}

bool evaluateAndCorrectPairData(int dc1, int dc2, float v1, float v2, float i1, float i2, float& out_I, float& out_IR) {
    out_I = std::max(i1, i2);
    out_IR = 0.0f;

    float slope1 = 0.0f, slope2 = 0.0f;
    bool linear1 = isDutyCycleLinearRegion(dc1, slope1);
    bool linear2 = isDutyCycleLinearRegion(dc2, slope2);

    // Exclusion 1: Both duty cycles in severe dead region
    if (!linear1 && !linear2 && (i1 < MEASURABLE_CURRENT_THRESHOLD && i2 < MEASURABLE_CURRENT_THRESHOLD)) {
        Serial.printf("Excluding pair (DC %d, %d): Both in non-linear dead region.\n", dc1, dc2);
        return false;
    }

    float deltaI_meas = std::fabs(i2 - i1);
    float deltaI_model = std::fabs(estimateCurrent(dc2) - estimateCurrent(dc1));

    // Exclusion 2: High-end driver saturation divergence protection
    if (dc1 > (int)(0.6f * MAX_DUTY_CYCLE) && dc2 > (int)(0.6f * MAX_DUTY_CYCLE) && deltaI_meas < 0.12f) {
        Serial.printf("Excluding high-end pair (DC %d, %d): High-end driver saturation risk (Delta I=%.4fA).\n",
                      dc1, dc2, deltaI_meas);
        return false;
    }

    if (deltaI_meas < REMEASURE_MIN_CURRENT_DIFF && deltaI_model < REMEASURE_MIN_CURRENT_DIFF) {
        Serial.printf("Excluding pair (DC %d, %d): Delta I too small (measured=%.4fA, model=%.4fA).\n",
                      dc1, dc2, deltaI_meas, deltaI_model);
        return false;
    }

    float deltaV = std::fabs(v1 - v2);

    float deltaI_eff = deltaI_meas;
    if (linear1 && linear2 && deltaI_model > 0.01f) {
        deltaI_eff = (1.0f - MODEL_CORRECTION_WEIGHT) * deltaI_meas + MODEL_CORRECTION_WEIGHT * deltaI_model;
    }

    if (deltaI_eff < 1e-4f) return false;

    float ir_calc = deltaV / deltaI_eff;

    // Exclusion 3: Physical bounds & divergence check against baseline
    if (ir_calc < MIN_VALID_RESISTANCE || ir_calc > REMEASURE_MAX_VALID_IR) {
        Serial.printf("Excluding pair (DC %d, %d): Calculated IR %.4f out of physical bounds.\n", dc1, dc2, ir_calc);
        return false;
    }

    float baseIR = regressedInternalResistanceIntercept;
    if (baseIR > 0.01f && baseIR < 1.0f) {
        if (ir_calc > 3.0f * baseIR + 0.25f && out_I > 0.5f) {
            Serial.printf("Excluding divergent high-end pair IR %.4f vs baseline %.4f at I=%.3fA\n",
                          ir_calc, baseIR, out_I);
            return false;
        }
    }

    out_IR = ir_calc;
    return true;
}

void handlePairGeneration() {
    std::vector<DutyPair> catPairs;
    generateCategorizedDutyPairs(catPairs, MAX_RESISTANCE_POINTS / 2);

    dutyCyclePairs.clear();
    for (const auto& cp : catPairs) {
        dutyCyclePairs.push_back({cp.lowDC, cp.highDC});
    }

    currentIRState = IR_STATE_MEASURE_L_UL;
    pairIndex = 0;
    measureStep = 0;
}

void handleMeasureLoadedUnloaded() {
    if (pairIndex >= (int)dutyCyclePairs.size()) {
        currentIRState = IR_STATE_MEASURE_PAIRS;
        pairIndex = 0;
        measureStep = 0;
        return;
    }
    switch (measureStep) {
        case 0:
            getSingleMeasurement(dutyCyclePairs[pairIndex].second, IR_STATE_MEASURE_L_UL);
            measureStep = 1;
            break;
        case 1:
            voltagesLoaded.push_back(currentMeasurement.voltage);
            currentsLoaded.push_back(currentMeasurement.current);
            ir_dutyCycles.push_back(static_cast<float>(currentMeasurement.dutyCycle));
            getSingleMeasurement(0, IR_STATE_MEASURE_L_UL);
            measureStep = 2;
            break;
        case 2:
            {
                float loadedVoltage = voltagesLoaded.back();
                float loadedCurrent = currentsLoaded.back();
                int dc = dutyCyclePairs[pairIndex].second;

                float unloadedVoltage = currentMeasurement.voltage;
                float unloadedCurrent = currentMeasurement.current;

                float calcI = 0.0f, calcIR = 0.0f;
                bool valid = evaluateAndCorrectPairData(0, dc, unloadedVoltage, loadedVoltage, unloadedCurrent, loadedCurrent, calcI, calcIR);

                if (valid) {
                    WEB_LOCK();
                    storeResistanceData(calcI, calcIR, internalResistanceData, resistanceDataCount);
                    bubbleSort(internalResistanceData, resistanceDataCount);
                    WEB_UNLOCK();
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void handleMeasurePairs() {
    if (pairIndex >= (int)dutyCyclePairs.size()) {
        currentIRState = IR_STATE_COMPLETE;
        return;
    }
    switch (measureStep) {
        case 0:
            getSingleMeasurement(dutyCyclePairs[pairIndex].first, IR_STATE_MEASURE_PAIRS);
            measureStep = 1;
            break;
        case 1:
            voltagesLoaded.push_back(currentMeasurement.voltage);
            currentsLoaded.push_back(currentMeasurement.current);
            getSingleMeasurement(dutyCyclePairs[pairIndex].second, IR_STATE_MEASURE_PAIRS);
            measureStep = 2;
            break;
        case 2:
            {
                float v1 = voltagesLoaded.back();
                float i1 = currentsLoaded.back();
                int dc1 = dutyCyclePairs[pairIndex].first;

                float v2 = currentMeasurement.voltage;
                float i2 = currentMeasurement.current;
                int dc2 = dutyCyclePairs[pairIndex].second;

                float calcI = 0.0f, calcIR = 0.0f;
                bool valid = evaluateAndCorrectPairData(dc1, dc2, v1, v2, i1, i2, calcI, calcIR);

                if (valid) {
                    consecutiveInternalResistances.push_back(calcIR);
                    WEB_LOCK();
                    storeResistanceData(calcI, calcIR, internalResistanceDataPairs, resistanceDataCountPairs);
                    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);
                    WEB_UNLOCK();
                } else {
                    consecutiveInternalResistances.push_back(-1.0f);
                }
                pairIndex++;
                measureStep = 0;
            }
            break;
    }
}

void completeResistanceMeasurement() {
    WEB_LOCK();
    bubbleSort(internalResistanceData, resistanceDataCount);
    bubbleSort(internalResistanceDataPairs, resistanceDataCountPairs);

    auto compute_spacing_threshold = [](float data[][2], int n) -> float {
        if (n < 2) return DISTRIBUTE_ERROR_DEFAULT_SPACING;
        float minX = data[0][0], maxX = data[n-1][0];
        float avgSpacing = (maxX - minX) / std::max(1, n-1);
        return std::max(DISTRIBUTE_ERROR_MIN_SPACING, avgSpacing * DISTRIBUTE_ERROR_SPACING_MULTIPLIER);
    };

    distribute_error(internalResistanceData, resistanceDataCount, compute_spacing_threshold(internalResistanceData, resistanceDataCount), DISTRIBUTE_ERROR_SD_MULTIPLIER);
    distribute_error(internalResistanceDataPairs, resistanceDataCountPairs, compute_spacing_threshold(internalResistanceDataPairs, resistanceDataCountPairs), DISTRIBUTE_ERROR_SD_MULTIPLIER);

    if (resistanceDataCount >= 2) {
        performLinearRegression(internalResistanceData, resistanceDataCount, regressedInternalResistanceSlope, regressedInternalResistanceIntercept);
    }
    if (resistanceDataCountPairs >= 2) {
        performLinearRegression(internalResistanceDataPairs, resistanceDataCountPairs, regressedInternalResistancePairsSlope, regressedInternalResistancePairsIntercept);
    }
    WEB_UNLOCK();
    isMeasuringResistance = false;
    applyDuty(0);
    currentIRState = IR_STATE_IDLE;
}

void bubbleSort(float data[][2], int n) {
    for (int i = 0; i < n - 1; i++) {
        bool swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            if (data[j][0] > data[j + 1][0]) {
                float temp0 = data[j][0], temp1 = data[j][1];
                data[j][0] = data[j + 1][0]; data[j][1] = data[j + 1][1];
                data[j + 1][0] = temp0; data[j + 1][1] = temp1;
                swapped = true;
            }
        }
        if (!swapped) break;
    }
}

void storeResistanceData(float current, float resistance, float dataArray[MAX_RESISTANCE_POINTS][2], int& count) {
    if (count >= MAX_RESISTANCE_POINTS) return;
    if (!std::isfinite(current) || !std::isfinite(resistance)) return;
    if (resistance > MIN_VALID_RESISTANCE && resistance < 1000.0f) {
        dataArray[count][0] = current;
        dataArray[count][1] = resistance;
        count++;
    }
}

int findClosestIndex(float data[][2], int count, float targetCurrent) {
    if (count == 0) return 0;
    int low = 0, high = count - 1;
    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (std::fabs(data[mid][0] - targetCurrent) < 1e-6f) return mid;
        else if (data[mid][0] < targetCurrent) low = mid + 1;
        else high = mid - 1;
    }
    if (low >= count) return high;
    if (high < 0) return low;
    return (std::fabs(targetCurrent - data[high][0]) < std::fabs(data[low][0] - targetCurrent)) ? high : low;
}

void insertDataPoint(float data[][2], int& count, float current, float resistance, int index) {
    if (count >= MAX_RESISTANCE_POINTS || index < 0 || index > count) return;
    for (int i = count; i > index; --i) {
        data[i][0] = data[i - 1][0]; data[i][1] = data[i - 1][1];
    }
    data[index][0] = current; data[index][1] = resistance;
    count++;
}

void removeDataPoint(float data[][2], int& count, int index) {
    if (index < 0 || index >= count) return;
    for (int i = index; i < count - 1; ++i) {
        data[i][0] = data[i + 1][0]; data[i][1] = data[i + 1][1];
    }
    count--;
}

void storeOrAverageResistanceData(float current, float resistance, float data[][2], int& count) {
    if (!std::isfinite(current) || !std::isfinite(resistance) || resistance <= MIN_VALID_RESISTANCE || resistance >= 1000.0f || current < 0.0f) return;
    if (count < MAX_RESISTANCE_POINTS) {
        int insertIndex = 0;
        while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
        insertDataPoint(data, count, current, resistance, insertIndex);
        return;
    }
    int closestIndex = findClosestIndex(data, count, current);
    const float CLOSE_TOLERANCE = 1e-3f * std::max(1.0f, current);
    if (std::fabs(data[closestIndex][0] - current) <= CLOSE_TOLERANCE) {
        const float alpha = 0.5f;
        data[closestIndex][1] = alpha * resistance + (1.0f - alpha) * data[closestIndex][1];
        data[closestIndex][0] = alpha * current + (1.0f - alpha) * data[closestIndex][0];
        return;
    }
    int evictIndex = 0;
    float maxGap = -1.0f;
    for (int i = 0; i < count; ++i) {
        float leftGap = (i > 0) ? data[i][0] - data[i-1][0] : 0.0f;
        float rightGap = (i < count-1) ? data[i+1][0] - data[i][0] : 0.0f;
        float localGap = std::max(leftGap, rightGap);
        if (localGap > maxGap) { maxGap = localGap; evictIndex = i; }
    }
    removeDataPoint(data, count, evictIndex);
    int insertIndex = 0;
    while (insertIndex < count && data[insertIndex][0] < current) insertIndex++;
    insertDataPoint(data, count, current, resistance, insertIndex);
}

float computeMedian(std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    size_t n = v.size();
    size_t mid = n / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float med = v[mid];
    if (n % 2 == 0) {
        auto it = std::max_element(v.begin(), v.begin() + mid);
        med = (med + *it) / 2.0f;
    }
    return med;
}

void distribute_error(float data[][2], int count, float spacing_threshold, float error_threshold_multiplier) {
    if (count < DISTRIBUTE_ERROR_MIN_NEIGHBORHOOD_SIZE) return;
    static std::vector<float> res;
    res.reserve(MAX_RESISTANCE_POINTS);
    for (int i = 0; i <= count - DISTRIBUTE_ERROR_MIN_NEIGHBORHOOD_SIZE; ++i) {
        for (int j = i + (DISTRIBUTE_ERROR_MIN_NEIGHBORHOOD_SIZE - 1); j < count; ++j) {
            if (data[j][0] - data[i][0] <= spacing_threshold) {
                res.clear();
                for (int k = i; k <= j; ++k) res.push_back(data[k][1]);
                if ((int)res.size() >= DISTRIBUTE_ERROR_MIN_NEIGHBORHOOD_SIZE) {
                    float median = computeMedian(res), sumSq = 0.0f;
                    for (float r : res) { float d = r - median; sumSq += d * d; }
                    float stdDev = std::sqrt(sumSq / res.size()), errorThreshold = error_threshold_multiplier * stdDev;
                    const float alpha = DISTRIBUTE_ERROR_SMOOTHING_ALPHA;
                    for (int k = i; k <= j; ++k) {
                        if (std::fabs(data[k][1] - median) > errorThreshold && stdDev > 1e-9f) {
                            data[k][1] = alpha * median + (1.0f - alpha) * data[k][1];
                        }
                    }
                    i = j; break;
                }
            } else break;
        }
    }
}

bool performLinearRegression(float data[][2], int count, float& slope, float& intercept) {
    if (count < 2) return false;
#ifndef MOCK_TEST_DISABLED // Use it even in mock if possible, but let's provide a fallback
    std::vector<float> x(count), y(count);
    for (int i = 0; i < count; ++i) {
        x[i] = data[i][0];
        y[i] = data[i][1];
    }
    AdvancedPolynomialFitter fitter;
    std::vector<float> coeffs = fitter.fitPolynomialLebesgue(x, y, 1);
    if (coeffs.size() >= 2) {
        intercept = coeffs[0];
        slope = coeffs[1];
    } else {
        return false;
    }
#else
    // Simple least squares fallback for mock or if fitter fails
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < count; i++) {
        sumX += data[i][0];
        sumY += data[i][1];
        sumXY += (double)data[i][0] * data[i][1];
        sumX2 += (double)data[i][0] * data[i][0];
    }
    double denominator = (count * sumX2 - sumX * sumX);
    if (std::abs(denominator) < 1e-9) return false;
    slope = (float)((count * sumXY - sumX * sumY) / denominator);
    intercept = (float)((sumY - (double)slope * sumX) / count);
#endif

    // Calculate Standard Error of the Regression to determine fit quality / error
    double ss_resid = 0.0;
    for (int i = 0; i < count; i++) {
        double pred = (double)slope * data[i][0] + intercept;
        double res = data[i][1] - pred;
        ss_resid += res * res;
    }
    double std_error = 0.0;
    if (count > 2) {
        std_error = std::sqrt(ss_resid / (count - 2));
    } else if (count == 2) {
        std_error = std::sqrt(ss_resid / 1.0);
    }
    Serial.printf("Linear Regression Fit: Slope = %.4f, Intercept = %.4f, Standard Error = %.4f Ohms (n = %d)\n", slope, intercept, (float)std_error, count);
    return true;
}
