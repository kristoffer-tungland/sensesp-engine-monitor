#include <Arduino.h>

#include "sensesp/sensors/digital_pcnt_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

constexpr uint8_t kCoolantTempPin = 34;
constexpr uint8_t kOilPressurePin = 35;
constexpr uint8_t kRpmPulsePin = 4;

constexpr unsigned int kAnalogReadIntervalMs = 500;
constexpr unsigned int kRpmReadIntervalMs = 500;
constexpr float kMillivoltsPerVolt = 1000.0f;
constexpr float kMillisecondsPerSecond = 1000.0f;
constexpr float kCoolantTempCelsiusPerVolt = 20.0f;
constexpr float kCoolantKelvinOffset = 273.15f;
constexpr float kOilPressurePaPerVolt = 100000.0f;
constexpr float kOilPressureOffsetPa = 0.0f;
constexpr float kRpmPulsesPerRevolution = 2.0f;

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("engine-monitor")->get_app();

  auto* coolant_input = new RepeatSensor<float>(kAnalogReadIntervalMs, []() {
    return analogReadMilliVolts(kCoolantTempPin) / kMillivoltsPerVolt;
  });
  auto* coolant_temp_kelvin = new Linear(kCoolantTempCelsiusPerVolt,
                                         kCoolantKelvinOffset,
                                         "/coolant/linear");
  coolant_input->connect_to(coolant_temp_kelvin)->connect_to(
      new SKOutputFloat("propulsion.main.coolantTemperature"));

  auto* oil_input = new RepeatSensor<float>(kAnalogReadIntervalMs, []() {
    return analogReadMilliVolts(kOilPressurePin) / kMillivoltsPerVolt;
  });
  auto* oil_pressure_pa = new Linear(kOilPressurePaPerVolt,
                                     kOilPressureOffsetPa,
                                     "/oil_pressure/linear");
  oil_input->connect_to(oil_pressure_pa)->connect_to(
      new SKOutputFloat("propulsion.main.oilPressure"));

  auto* rpm_counter = new DigitalInputPcntCounter(
      kRpmPulsePin, INPUT_PULLUP, RISING, kRpmReadIntervalMs);
  auto* revolutions_per_second =
      new LambdaTransform<int, float>([](int pulses_in_interval) {
        return (kMillisecondsPerSecond * pulses_in_interval) /
               (kRpmReadIntervalMs * kRpmPulsesPerRevolution);
      });
  rpm_counter->connect_to(revolutions_per_second)->connect_to(
      new SKOutputFloat("propulsion.main.revolutions"));
}

void loop() { event_loop()->tick(); }
