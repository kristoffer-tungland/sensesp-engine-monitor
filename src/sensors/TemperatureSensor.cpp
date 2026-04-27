#include "sensors/TemperatureSensor.hpp"

#include <cmath>

TemperatureSensor::TemperatureSensor(uint8_t pin, uint8_t sensor_count,
                                     uint32_t refresh_ms)
    : one_wire_(pin),
      temp_bus_(&one_wire_),
      sensor_count_(sensor_count),
      refresh_ms_(refresh_ms),
      cached_kelvin_(sensor_count, NAN) {}

void TemperatureSensor::begin() { temp_bus_.begin(); }

float TemperatureSensor::read_kelvin(uint8_t index) {
  const uint32_t now = millis();
  bool should_refresh = false;

  portENTER_CRITICAL(&mux_);
  if (last_poll_ms_ == 0 || (now - last_poll_ms_) >= refresh_ms_) {
    last_poll_ms_ = now;
    should_refresh = true;
  }
  portEXIT_CRITICAL(&mux_);

  if (should_refresh) {
    refresh_cache();
  }

  portENTER_CRITICAL(&mux_);
  const float value = cached_kelvin_[index];
  portEXIT_CRITICAL(&mux_);
  return value;
}

void TemperatureSensor::refresh_cache() {
  temp_bus_.requestTemperatures();

  std::vector<float> samples(sensor_count_);
  for (uint8_t i = 0; i < sensor_count_; i++) {
    const float celsius = temp_bus_.getTempCByIndex(i);
    samples[i] = (celsius == DEVICE_DISCONNECTED_C) ? NAN : celsius + 273.15f;
  }

  portENTER_CRITICAL(&mux_);
  cached_kelvin_ = samples;
  portEXIT_CRITICAL(&mux_);
}
