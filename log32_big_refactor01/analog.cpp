#include "analog.h"
#include "config.h"

// --- ADC Calibration Data ---
// These are required by the ESP32 ADC driver.
adc_calibration_data_t adc1_cal_data[ADC1_CHANNEL_COUNT];
bool adc1_cal_initialized[ADC1_CHANNEL_COUNT] = { false };

// --- Function Implementations ---

int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
    int channel = get_adc1_channel(pin);
    if (channel < 0) {
        return -1; // Not an ADC1 pin
    }

    if (!adc1_cal_initialized[channel]) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten((adc1_channel_t)channel, attenuation);
        adc1_cal_data[channel].adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 1100, adc1_cal_data[channel].adc_chars);
        adc1_cal_initialized[channel] = true;
    } else if (adc1_cal_data[channel].current_atten != attenuation) {
        // Re-initialize if attenuation has changed
        adc1_config_channel_atten((adc1_channel_t)channel, attenuation);
        esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 1100, adc1_cal_data[channel].adc_chars);
        adc1_cal_data[channel].current_atten = attenuation;
    }

    uint32_t raw_reading = 0;
    for (int i = 0; i < oversampling; i++) {
        raw_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    raw_reading /= oversampling;

    return esp_adc_cal_raw_to_voltage(raw_reading, adc1_cal_data[channel].adc_chars);
}

// Reads the current from the shunt resistor and returns it in milliamps.
float readCurrent() {
    double sumAnalogValuesCurrent = 0;
    for (int i = 0; i < CURRENT_SHUNT_OVERSAMPLING; ++i) {
        sumAnalogValuesCurrent += analogReadMillivolts(CURRENT_SHUNT_PIN, CURRENT_SHUNT_ATTENUATION, 1); // Oversampling handled in the loop
    }
    double voltageAcrossShunt = (sumAnalogValuesCurrent / CURRENT_SHUNT_OVERSAMPLING) - CURRENT_SHUNT_PIN_ZERO_OFFSET;
    return (voltageAcrossShunt / CURRENT_SHUNT_RESISTANCE);
}

// Reads the battery voltage and returns it in millivolts.
float readVoltage(float vcc) {
    double sumAnalogValuesVoltage = 0;
    for (int i = 0; i < VOLTAGE_OVERSAMPLING; ++i) {
        sumAnalogValuesVoltage += analogReadMillivolts(VOLTAGE_READ_PIN, VOLTAGE_ATTENUATION, 1); // Oversampling handled in the loop
    }
    double avg_millivolts = sumAnalogValuesVoltage / VOLTAGE_OVERSAMPLING;
    return (vcc * MAIN_VCC_RATIO) - avg_millivolts;
}