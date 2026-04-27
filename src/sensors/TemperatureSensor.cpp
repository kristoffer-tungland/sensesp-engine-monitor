#include "sensors/TemperatureSensor.hpp"

#include <cmath>

#include "esp_log.h"

static const char* kTag = "TemperatureSensor";

TemperatureSensor::TemperatureSensor(uint8_t pin, uint8_t sensor_count,
                                     uint32_t refresh_ms)
    : one_wire_(pin),
      temp_bus_(&one_wire_),
      sensor_count_(sensor_count),
      refresh_ms_(refresh_ms),
      cached_kelvin_(sensor_count, NAN) {}

void TemperatureSensor::begin() {
  ESP_LOGI(kTag, "Initializing Dallas bus...");
  temp_bus_.begin();
  const uint8_t found = temp_bus_.getDeviceCount();
  const uint8_t parasites = temp_bus_.isParasitePowerMode();
  ESP_LOGI(kTag, "Dallas bus ready: %d device(s) found, parasite power: %s",
           found, parasites ? "YES" : "NO");
  if (found == 0) {
    ESP_LOGW(kTag,
             "No devices found! Check wiring: data on pin, 4.7k pull-up to "
             "3.3V, GND connected.");
  }
}

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
    ESP_LOGD(kTag, "Cache stale at %lu ms, refreshing...", (unsigned long)now);
    refresh_cache();
  }

  portENTER_CRITICAL(&mux_);
  const float value = cached_kelvin_[index];
  portEXIT_CRITICAL(&mux_);

  ESP_LOGD(kTag, "read_kelvin(%d) = %.2f K", index, value);
  return value;
}

void TemperatureSensor::refresh_cache() {
  ESP_LOGD(kTag, "Requesting temperatures from %d sensor(s)...", sensor_count_);
  temp_bus_.requestTemperatures();

  std::vector<float> samples(sensor_count_);
  for (uint8_t i = 0; i < sensor_count_; i++) {
    const float celsius = temp_bus_.getTempCByIndex(i);
    if (celsius == DEVICE_DISCONNECTED_C) {
      ESP_LOGW(kTag, "Sensor[%d]: DEVICE_DISCONNECTED", i);
      samples[i] = NAN;
    } else {
      samples[i] = celsius + 273.15f;
      ESP_LOGI(kTag, "Sensor[%d]: %.2f C  (%.2f K)", i, celsius, samples[i]);
    }
  }

  portENTER_CRITICAL(&mux_);
  cached_kelvin_ = samples;
  portEXIT_CRITICAL(&mux_);
}
