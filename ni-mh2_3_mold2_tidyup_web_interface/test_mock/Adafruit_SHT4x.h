#ifndef ADAFRUIT_SHT4X_H
#define ADAFRUIT_SHT4X_H

typedef enum {
  SHT4X_HIGH_PRECISION,
  SHT4X_MED_PRECISION,
  SHT4X_LOW_PRECISION,
} sht4x_precision_t;

typedef enum {
  SHT4X_NO_HEATER,
  SHT4X_HIGH_HEATER_1S,
  SHT4X_HIGH_HEATER_100MS,
  SHT4X_MED_HEATER_1S,
  SHT4X_MED_HEATER_100MS,
  SHT4X_LOW_HEATER_1S,
  SHT4X_LOW_HEATER_100MS,
} sht4x_heater_t;

class Adafruit_SHT4x {
public:
    bool begin() { return true; }
    void setPrecision(sht4x_precision_t precision) {}
    void setHeater(sht4x_heater_t heater) {}
    bool getEvent(void* temp, void* humidity) { return true; }
};

#endif
