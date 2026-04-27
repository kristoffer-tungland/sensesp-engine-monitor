#include "sensors/TachRpmTracker.hpp"

static volatile uint32_t s_pulse_count = 0;
static portMUX_TYPE s_pulse_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR on_tach_pulse() {
  portENTER_CRITICAL_ISR(&s_pulse_mux);
  s_pulse_count++;
  portEXIT_CRITICAL_ISR(&s_pulse_mux);
}

TachRpmTracker::TachRpmTracker(uint8_t tach_pin, float pulses_per_revolution,
                               float running_threshold_rpm)
    : tach_pin_(tach_pin),
      pulses_per_revolution_(pulses_per_revolution),
      running_threshold_rpm_(running_threshold_rpm) {}

void TachRpmTracker::begin() {
  pinMode(tach_pin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tach_pin_), on_tach_pulse, RISING);
}

float TachRpmTracker::sample_rpm(uint32_t interval_ms) {
  portENTER_CRITICAL(&s_pulse_mux);
  const uint32_t pulses = s_pulse_count;
  s_pulse_count = 0;
  portEXIT_CRITICAL(&s_pulse_mux);

  const float seconds = static_cast<float>(interval_ms) / 1000.0f;
  const float rps = (static_cast<float>(pulses) / pulses_per_revolution_) / seconds;
  latest_rpm_ = rps * 60.0f;

  if (latest_rpm_ > running_threshold_rpm_) {
    engine_run_seconds_ += static_cast<double>(interval_ms) / 1000.0;
  }

  return latest_rpm_;
}

float TachRpmTracker::rps() const { return latest_rpm_ / 60.0f; }

bool TachRpmTracker::is_running() const {
  return latest_rpm_ > running_threshold_rpm_;
}

float TachRpmTracker::engine_hours() const {
  return static_cast<float>(engine_run_seconds_ / 3600.0);
}
