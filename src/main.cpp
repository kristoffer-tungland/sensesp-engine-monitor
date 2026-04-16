#include <Arduino.h>

#include "sensesp/sensors/digital_pcnt_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

constexpr uint8_t kCoolantTempPin = 34;
constexpr uint8_t kOilPressurePin = 35;
constexpr uint8_t kRpmPulsePin = 4;

constexpr unsigned int kAnalogReadIntervalMs = 500;
constexpr unsigned int kRpmReadIntervalMs = 500;

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("engine-monitor")->get_app();

  auto* coolant_input = new RepeatSensor<float>(kAnalogReadIntervalMs, []() {
    return analogReadMilliVolts(kCoolantTempPin) / 1000.0f;
  });
  auto* coolant_temp_kelvin = new Linear(20.0f, 273.15f, "/coolant/linear");
  coolant_input->connect_to(coolant_temp_kelvin)->connect_to(
      new SKOutputFloat("propulsion.main.temperature"));

  auto* oil_input = new RepeatSensor<float>(kAnalogReadIntervalMs, []() {
    return analogReadMilliVolts(kOilPressurePin) / 1000.0f;
  });
  auto* oil_pressure_pa = new Linear(100000.0f, 0.0f, "/oil_pressure/linear");
  oil_input->connect_to(oil_pressure_pa)->connect_to(
      new SKOutputFloat("propulsion.main.oilPressure"));

  auto* rpm_counter = new DigitalInputPcntCounter(
      kRpmPulsePin, INPUT_PULLUP, RISING, kRpmReadIntervalMs);
  auto* rpm_frequency_hz = new Frequency(1.0f / 60.0f, "/rpm/frequency");
  rpm_counter->connect_to(rpm_frequency_hz)->connect_to(
      new SKOutput<float>("propulsion.main.revolutions"));
}

void loop() { event_loop()->tick(); }
