#ifndef ADC_DMA_H
#define ADC_DMA_H

#include <Arduino.h>
#ifndef MOCK_TEST
#include "esp_adc/adc_continuous.h"
#include "esp_adc_cal.h"
#endif

// Define the ADC channels we are interested in
enum AdcChannelIndex {
    ADC_IDX_THERM1 = 0,    // GPIO36, ADC1_CHANNEL_0
    ADC_IDX_THERM_VCC = 1, // GPIO35, ADC1_CHANNEL_7
    ADC_IDX_VOLTAGE = 2,   // GPIO39, ADC1_CHANNEL_3
    ADC_IDX_CURRENT = 3,   // GPIO34, ADC1_CHANNEL_6
    ADC_CH_COUNT = 4
};

// Snapshot structure for superior oversampling
struct AdcSnapshot {
    uint64_t sum;
    uint32_t count;
};

// Function prototypes
void setupAdcDma();
void processAdcDma();

// Direct retrieval of latest single-batch averages (legacy/simple support)
float getAdcMillivolts(AdcChannelIndex idx);
uint32_t getAdcRawAverage(AdcChannelIndex idx);

// Snapshot retrieval for cumulative oversampling across arbitrary windows
void getAdcSnapshot(AdcChannelIndex idx, AdcSnapshot &snapshot);
uint32_t calculateSnapshotAverage(const AdcSnapshot &old_s, const AdcSnapshot &new_s);
float snapshotToMillivolts(AdcChannelIndex idx, uint32_t avg_raw);

#endif // ADC_DMA_H
