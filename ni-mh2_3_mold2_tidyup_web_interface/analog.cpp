// analog.cpp

#include "analog.h"
#include "adc_dma.h"
#include "definitions.h"

adc_calibration_data_t adc1_cal_data[ADC1_CHANNEL_COUNT];
bool adc1_cal_initialized[ADC1_CHANNEL_COUNT] = {false};

int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
  // Try DMA system first
  if (pin == THERMISTOR_PIN_1) {
    return (int)getAdcMillivolts(ADC_IDX_THERM1);
  } else if (pin == THERMISTOR_VCC_PIN) {
    return (int)getAdcMillivolts(ADC_IDX_THERM_VCC);
  } else if (pin == VOLTAGE_READ_PIN) {
    return (int)getAdcMillivolts(ADC_IDX_VOLTAGE);
  } else if (pin == CURRENT_SHUNT_PIN) {
    return (int)getAdcMillivolts(ADC_IDX_CURRENT);
  }

  // Fallback to synchronous reading for other pins
  int adc1_chan = get_adc1_channel(pin);
  if (adc1_chan == -1) {
    log_e("analogReadMillivolts", "Invalid pin for ADC1: %d", pin);
    return -1;
  }

  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12); // Using 12-bit resolution
  esp_err_t err = adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);
  if (err != ESP_OK) {
    log_e("analogReadMillivolts", "Error configuring attenuation for pin %d: %s", pin, esp_err_to_name(err));
    return -1;
  }

  // Initialize calibration data if not already initialized or if attenuation changed
  if (!adc1_cal_initialized[adc1_chan] || adc1_cal_data[adc1_chan].current_atten != attenuation) {
    if (adc1_cal_data[adc1_chan].adc_chars != NULL) {
      free(adc1_cal_data[adc1_chan].adc_chars);
    }
    adc1_cal_data[adc1_chan].adc_chars = (esp_adc_cal_characteristics_t *)malloc(sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 3300, adc1_cal_data[adc1_chan].adc_chars); // Assuming 3.3V Vref
    adc1_cal_data[adc1_chan].current_atten = attenuation;
    adc1_cal_initialized[adc1_chan] = true;
  }

  int raw_adc = 0;
  if (oversampling > 1) {
    long long sum = 0;
    for (int i = 0; i < oversampling; i++) {
      sum += adc1_get_raw((adc1_channel_t)adc1_chan);
    }
    raw_adc = sum / oversampling;
  } else {
    raw_adc = adc1_get_raw((adc1_channel_t)adc1_chan);
  }

  if (adc1_cal_initialized[adc1_chan]) {
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc1_cal_data[adc1_chan].adc_chars);
    return (int)voltage; // voltage is in mV
  } else {
    log_w("analogReadMillivolts", "Calibration not initialized for pin %d, returning raw value scaled (approximate).", pin);
    return (raw_adc * 3300) / 4095;
  }
}
