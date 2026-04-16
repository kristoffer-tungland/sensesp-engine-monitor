#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <cmath>

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
constexpr uint8_t kOneWireDataPin = 25;

constexpr unsigned int kAnalogReadIntervalMs = 500;
constexpr unsigned int kRpmReadIntervalMs = 500;
constexpr unsigned int kDallasReadIntervalMs = 10000;
constexpr float kMillivoltsPerVolt = 1000.0f;
constexpr float kMillisecondsPerSecond = 1000.0f;
constexpr float kCoolantTempCelsiusPerVolt = 20.0f;
constexpr float kCoolantKelvinOffset = 273.15f;
constexpr float kOilPressurePaPerVolt = 100000.0f;
constexpr float kOilPressureOffsetPa = 0.0f;
constexpr float kRpmPulsesPerRevolution = 2.0f;
constexpr float kCelsiusToKelvinOffset = 273.15f;

constexpr int kDallasSensorCount = 4;
constexpr uint64_t kDallasSensorAddresses[kDallasSensorCount] = {
    0x2c3c18e381b14128ULL,
    0xf43c9ae38186c228ULL,
    0x5c3c09e3810b9328ULL,
    0x443c7de381c5a828ULL,
};

OneWire one_wire_bus(kOneWireDataPin);
DallasTemperature dallas_bus(&one_wire_bus);
float dallas_temperatures_c[kDallasSensorCount] = {NAN, NAN, NAN, NAN};
DeviceAddress dallas_device_addresses[kDallasSensorCount];

void address_from_u64(uint64_t source, DeviceAddress output) {
  for (uint8_t i = 0; i < 8; i++) {
    output[i] = static_cast<uint8_t>((source >> (8 * i)) & 0xff);
  }
}

float celsius_to_kelvin(float celsius) {
  if (std::isnan(celsius)) {
    return NAN;
  }
  return celsius + kCelsiusToKelvinOffset;
}

LambdaTransform<float, float>* new_celsius_to_kelvin_transform() {
  return new LambdaTransform<float, float>(
      [](float celsius) { return celsius_to_kelvin(celsius); });
}

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("engine-monitor")->get_app();

  dallas_bus.begin();
  dallas_bus.setWaitForConversion(true);
  for (int index = 0; index < kDallasSensorCount; index++) {
    address_from_u64(kDallasSensorAddresses[index], dallas_device_addresses[index]);
  }
  event_loop()->onRepeat(kDallasReadIntervalMs, []() {
    dallas_bus.requestTemperatures();
    for (int index = 0; index < kDallasSensorCount; index++) {
      auto celsius = dallas_bus.getTempC(dallas_device_addresses[index]);
      dallas_temperatures_c[index] =
          (celsius == DEVICE_DISCONNECTED_C) ? NAN : celsius;
    }
  });

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

  auto* temp1_c = new RepeatSensor<float>(kDallasReadIntervalMs,
                                          []() { return dallas_temperatures_c[0]; });
  temp1_c
      ->connect_to(new_celsius_to_kelvin_transform())
      ->connect_to(
          new SKOutputFloat("propulsion.main.auxiliaryTemperature.rawWaterElbow"));

  auto* temp2_c = new RepeatSensor<float>(kDallasReadIntervalMs, []() {
    return dallas_temperatures_c[1];
  });
  temp2_c
      ->connect_to(new_celsius_to_kelvin_transform())
      ->connect_to(new SKOutputFloat("electrical.alternators.main.temperature"));

  auto* temp3_c = new RepeatSensor<float>(kDallasReadIntervalMs, []() {
    return dallas_temperatures_c[2];
  });
  temp3_c
      ->connect_to(new_celsius_to_kelvin_transform())
      ->connect_to(
          new SKOutputFloat("propulsion.main.auxiliaryTemperature.coolantArea"));

  auto* temp4_c = new RepeatSensor<float>(kDallasReadIntervalMs, []() {
    return dallas_temperatures_c[3];
  });
  temp4_c
      ->connect_to(new_celsius_to_kelvin_transform())
      ->connect_to(
          new SKOutputFloat("propulsion.main.auxiliaryTemperature.thermostat"));
}

void loop() { event_loop()->tick(); }
