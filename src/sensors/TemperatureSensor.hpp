#ifndef SENSORS_TEMPERATURE_SENSOR_HPP_
#define SENSORS_TEMPERATURE_SENSOR_HPP_

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <vector>

class TemperatureSensor {
 public:
  TemperatureSensor(uint8_t pin, uint8_t sensor_count, uint32_t refresh_ms);

  void begin();

  // Returns the temperature in Kelvin for the given sensor index.
  // All sensors are batch-polled once per refresh_ms; subsequent calls within
  // the same interval return the cached value without re-polling the bus.
  float read_kelvin(uint8_t index);

 private:
  void refresh_cache();

  OneWire one_wire_;
  DallasTemperature temp_bus_;
  uint8_t sensor_count_;
  uint32_t refresh_ms_;
  std::vector<float> cached_kelvin_;
  uint32_t last_poll_ms_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

#endif
