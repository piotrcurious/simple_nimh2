// ThermistorSensor.cpp
#include "ThermistorSensor.h"
#include <Arduino.h>
#include "analog.h"

ThermistorSensor::ThermistorSensor(int thermistor1Pin, int thermistorVccPin, double thermistor1Offset)
    : thermistor1Pin(thermistor1Pin), thermistorVccPin(thermistorVccPin), thermistor1Offset(thermistor1Offset),
      vcc_millivolts(1000.0), thermistor1Value(25.0),thermistor2Value(25.0), thermistorDiffValue(0.0),
      thermistorVccValue(600.0), thermistor1RawMillivolts(300.0), lock(false), last_time(0) {}

void ThermistorSensor::begin() {
//    analogSetPinAttenuation(thermistor1Pin, ADC_0db);
    //analogSetPinAttenuation(thermistor2Pin, ADC_0db);
//    analogSetPinAttenuation(thermistorVccPin, ADC_0db);
}

void ThermistorSensor::read(double topThermistorTemperatureCelsius) {
    uint32_t current_time = millis();
    if ((current_time - last_time) > update_interval) {
        lock = true; // Set lock to prevent reads in loop

        double currentVCC = readVCCInternal(thermistorVccPin, 256);
        const float vcc_alpha = 0.1;

        thermistorVccValue = currentVCC; // do not average because it defines VCC for further readings and it needs to be accurate for the very moment
        //thermistorVccValue = (1.0 - vcc_alpha) * thermistorVccValue + vcc_alpha * currentVCC;
        vcc_millivolts = thermistorVccValue; // Read averaged VCC value

        adc_atten_t attenuationSetting = ADC_ATTEN_DB_11; // Example attenuation
        int oversamplingFactor = 16; // Example oversampling factor
        float rawMillivolts1 = analogReadMillivolts(thermistor1Pin, attenuationSetting, oversamplingFactor);

        //float rawMillivolts1 = analogReadMilliVolts(thermistor1Pin);

        // Calculate derived values
        double temp2 = readThermistorRelativeInternal(thermistor1Pin, topThermistorTemperatureCelsius, 64, thermistor1Offset); // Thermistor 2 (Bottom)
        double tempDiff = -topThermistorTemperatureCelsius + temp2; // Difference (Top - Bottom)

        // Average the readings
        const float alpha = 0.1; // Smoothing factor for the moving average
        thermistor1Value = (1.0 - alpha) * thermistor1Value + alpha * topThermistorTemperatureCelsius; // Assuming top temp comes from SHT4x
        thermistor2Value = (1.0 - alpha) * thermistor2Value + alpha * temp2;
        thermistorDiffValue = (1.0 - alpha) * thermistorDiffValue + alpha * tempDiff;
        thermistor1RawMillivolts = (1.0 - alpha) * thermistor1RawMillivolts + alpha * rawMillivolts1;

        lock = false; // Release lock
        last_time = current_time;
    }
}

double ThermistorSensor::getTemperature1() {
    return thermistor1Value;
}

double ThermistorSensor::getTemperature2() {
    return thermistor2Value;
}

double ThermistorSensor::getDifference() {
    return thermistorDiffValue;
}

double ThermistorSensor::getVCC() {
    return thermistorVccValue;
}

float ThermistorSensor::getRawMillivolts1() {
    return thermistor1RawMillivolts;
}


//void ThermistorSensor::setVccMillivolts(double vcc) {
//    vcc_millivolts = vcc;
//}

bool ThermistorSensor::isLocked() {
    return lock;
}

double ThermistorSensor::readVCCInternal(int pin, int numSamples) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
//        uint32_t analogValue = analogReadMilliVolts(pin);
          adc_atten_t attenuationSetting = ADC_ATTEN_DB_11; // Example attenuation
          int oversamplingFactor = 16; // Example oversampling factor
          uint32_t analogValue = analogReadMillivolts(pin,attenuationSetting, oversamplingFactor);

        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = sumAnalogValues / numSamples;
    return averageAnalogValue;
}

/*
double ThermistorSensor::readThermistorInternal(int pin, int numSamples) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = sumAnalogValues / numSamples;

    // Convert average analog reading to resistance
    double resistance = (averageAnalogValue * SERIESRESISTOR) / (vcc_millivolts - averageAnalogValue);
    return resistanceToTemperature(resistance);
}
*/


/*
double ThermistorSensor::readThermistorRelativeInternal(int pin, double topThermistorTemperatureCelsius, int numSamples, double resistance_offset) {
    if (numSamples <= 0) numSamples = 1; // Ensure at least one sample

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {
        uint32_t analogValue = analogReadMilliVolts(pin);
        if (analogValue == 0) {
            // Handle potential zero reading - indicates error
            return std::numeric_limits<double>::quiet_NaN(); // Return NaN (Not a Number) to indicate error
        }
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = sumAnalogValues / numSamples;
    double topThermistorResistance = temperatureToResistance(topThermistorTemperatureCelsius);
    // Convert average analog reading to resistance and add fixed offset for calibration
    double resistance = resistance_offset + (averageAnalogValue * topThermistorResistance) / (vcc_millivolts - averageAnalogValue);
    return resistanceToTemperature(resistance);
}

*/
double ThermistorSensor::readThermistorRelativeInternal(
    int pin, double topThermistorTemperatureCelsius, int numSamples, double offset) {

    if (numSamples <= 0) numSamples = 1;

    double sumAnalogValues = 0;
    for (int i = 0; i < numSamples; ++i) {

//        uint32_t analogValue = analogReadMilliVolts(pin);
    adc_atten_t attenuationSetting = ADC_ATTEN_DB_11; // Example attenuation
    int oversamplingFactor = 16; // Example oversampling factor
    uint32_t analogValue = analogReadMillivolts(pin,attenuationSetting,oversamplingFactor);

        if (analogValue == 0) return std::numeric_limits<double>::quiet_NaN();
        sumAnalogValues += analogValue;
    }
    double averageAnalogValue = (sumAnalogValues / numSamples) - offset;

    double vRatio = averageAnalogValue / ((vcc_millivolts*MAIN_VCC_RATIO) - averageAnalogValue);
    double logVRatio = log(vRatio);

    double topThermistorTemperatureKelvin = topThermistorTemperatureCelsius + 273.15;
    double beta = BCOEFFICIENT; // Assuming BCOEFFICIENT is defined

    double invBottomTempKelvin = (1.0 / topThermistorTemperatureKelvin) + (logVRatio / beta);
    double bottomThermistorTemperatureKelvin = 1.0 / invBottomTempKelvin;
    double bottomThermistorTemperatureCelsius = bottomThermistorTemperatureKelvin - 273.15;

    return bottomThermistorTemperatureCelsius;
}


// Helper function to convert resistance to temperature (Celsius) using Beta equation
double ThermistorSensor::resistanceToTemperature(double resistance) {
    double steinhart;
    steinhart = log(resistance / THERMISTORNOMINAL);           // (R/Ro)
    steinhart /= BCOEFFICIENT;                                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);           // + (1/To)  (To in Kelvin)
    steinhart = 1.0 / steinhart;                                   // Invert (Kelvin)
    return steinhart - 273.15;                                   // Convert to Celsius
}

// Helper function to convert temperature (Celsius) to resistance
double ThermistorSensor::temperatureToResistance(double temperatureCelsius) {
    double temperatureKelvin = temperatureCelsius + 273.15;
    return THERMISTORNOMINAL * exp(BCOEFFICIENT * (1.0 / temperatureKelvin - 1.0 / (TEMPERATURENOMINAL + 273.15)));
}

// Float version of map function for better precision in scaling
float ThermistorSensor::mapfInternal(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
