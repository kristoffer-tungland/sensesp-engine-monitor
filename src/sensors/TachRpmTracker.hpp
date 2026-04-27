#ifndef SENSORS_TACH_RPM_TRACKER_HPP_
#define SENSORS_TACH_RPM_TRACKER_HPP_

#include <Arduino.h>

class TachRpmTracker {
 public:
  TachRpmTracker(uint8_t tach_pin, float pulses_per_revolution,
                 float running_threshold_rpm);

  void begin();

  // Reads and resets the pulse counter, computes RPM, and accumulates engine
  // hours. Must be called at the interval_ms cadence used for RPM sampling.
  float sample_rpm(uint32_t interval_ms);

  float rps() const;
  bool is_running() const;
  float engine_hours() const;

 private:
  uint8_t tach_pin_;
  float pulses_per_revolution_;
  float running_threshold_rpm_;
  float latest_rpm_ = 0.0f;
  double engine_run_seconds_ = 0.0;
};

#endif
