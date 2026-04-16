#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_INA219.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

namespace {
constexpr uint8_t kDs18b20Pin = 25;
constexpr uint8_t kTachPin = 26;
constexpr uint8_t kI2cSdaPin = 21;
constexpr uint8_t kI2cSclPin = 22;

constexpr uint8_t kTempSensorCount = 4;
constexpr uint32_t kTempReadMs = 10000;
constexpr uint32_t kOilFuelReadMs = 1000;
constexpr uint32_t kRpmReadMs = 1000;
constexpr uint32_t kHoursReadMs = 5000;
constexpr float kSecondsPerHour = 3600.0f;

constexpr float kKelvinOffset = 273.15f;
constexpr float kPulsesPerRevolution = 1.0f;
constexpr float kRunningThresholdRpm = 100.0f;

constexpr float kOilMinShuntMv = 0.0f;
constexpr float kOilMaxShuntMv = 100.0f;
constexpr float kOilMaxBar = 10.0f;

constexpr float kFuelEmptyShuntMv = 0.0f;
constexpr float kFuelFullShuntMv = 100.0f;

OneWire* one_wire = nullptr;
DallasTemperature* temp_bus = nullptr;
Adafruit_INA219* oil_sensor = nullptr;
Adafruit_INA219* fuel_sensor = nullptr;

float latest_rpm = 0.0f;
double engine_run_seconds = 0.0;
volatile uint32_t tach_pulse_count = 0;
float cached_temps_kelvin[kTempSensorCount] = {NAN, NAN, NAN, NAN};
uint32_t last_temp_poll_ms = 0;
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE tach_mux = portMUX_INITIALIZER_UNLOCKED;

float clampf(float value, float min_value, float max_value) {
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}

float map_range(float input, float in_min, float in_max, float out_min,
                float out_max) {
  if (in_max == in_min) {
    return out_min;
  }
  const float t = (input - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

float read_temperature_kelvin(uint8_t index) {
  const uint32_t now = millis();
  bool should_refresh = false;

  portENTER_CRITICAL(&state_mux);
  if (last_temp_poll_ms == 0 || (now - last_temp_poll_ms) >= kTempReadMs) {
    last_temp_poll_ms = now;
    should_refresh = true;
  }
  portEXIT_CRITICAL(&state_mux);

  if (should_refresh) {
    float samples[kTempSensorCount];
    temp_bus->requestTemperatures();
    for (uint8_t i = 0; i < kTempSensorCount; i++) {
      const float celsius = temp_bus->getTempCByIndex(i);
      samples[i] = (celsius == DEVICE_DISCONNECTED_C) ? NAN : celsius + kKelvinOffset;
    }

    portENTER_CRITICAL(&state_mux);
    for (uint8_t i = 0; i < kTempSensorCount; i++) {
      cached_temps_kelvin[i] = samples[i];
    }
    portEXIT_CRITICAL(&state_mux);
  }

  portENTER_CRITICAL(&state_mux);
  const float value = cached_temps_kelvin[index];
  portEXIT_CRITICAL(&state_mux);
  return value;
}

void IRAM_ATTR on_tach_pulse() {
  portENTER_CRITICAL_ISR(&tach_mux);
  tach_pulse_count++;
  portEXIT_CRITICAL_ISR(&tach_mux);
}
}  // namespace

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.get_app();  // Initializes global app instance used by SensESP runtime.

  static OneWire one_wire_instance(kDs18b20Pin);
  static DallasTemperature temp_bus_instance(&one_wire_instance);
  static Adafruit_INA219 oil_sensor_instance(0x40);
  static Adafruit_INA219 fuel_sensor_instance(0x41);
  one_wire = &one_wire_instance;
  temp_bus = &temp_bus_instance;
  oil_sensor = &oil_sensor_instance;
  fuel_sensor = &fuel_sensor_instance;

  temp_bus->begin();
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  oil_sensor->begin();
  fuel_sensor->begin();
  pinMode(kTachPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kTachPin), on_tach_pulse, RISING);

  auto* raw_water_temp = new RepeatSensor<float>(kTempReadMs, []() {
    return read_temperature_kelvin(0);
  });
  auto* alternator_temp = new RepeatSensor<float>(kTempReadMs, []() {
    return read_temperature_kelvin(1);
  });
  auto* coolant_temp = new RepeatSensor<float>(kTempReadMs, []() {
    return read_temperature_kelvin(2);
  });
  auto* thermostat_temp = new RepeatSensor<float>(kTempReadMs, []() {
    return read_temperature_kelvin(3);
  });

  raw_water_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.rawWater"));
  alternator_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.alternator"));
  coolant_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.coolant"));
  thermostat_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.thermostat"));

  auto* oil_pressure = new RepeatSensor<float>(kOilFuelReadMs, []() {
    const float shunt_mv = oil_sensor->getShuntVoltage_mV();
    const float bar = map_range(shunt_mv, kOilMinShuntMv, kOilMaxShuntMv, 0.0f,
                                kOilMaxBar);
    return clampf(bar, 0.0f, kOilMaxBar);
  });
  oil_pressure->connect_to(new SKOutputFloat("propulsion.engine.oilPressure"));

  auto* fuel_level = new RepeatSensor<float>(kOilFuelReadMs, []() {
    const float shunt_mv = fuel_sensor->getShuntVoltage_mV();
    const float pct =
        map_range(shunt_mv, kFuelEmptyShuntMv, kFuelFullShuntMv, 0.0f, 1.0f);
    return clampf(pct, 0.0f, 1.0f);
  });
  fuel_level->connect_to(new SKOutputFloat("tanks.fuel.level"));

  auto* rpm_sensor = new RepeatSensor<float>(kRpmReadMs, []() {
    portENTER_CRITICAL(&tach_mux);
    const uint32_t pulses = tach_pulse_count;
    tach_pulse_count = 0;
    portEXIT_CRITICAL(&tach_mux);

    const float seconds = static_cast<float>(kRpmReadMs) / 1000.0f;
    const float rps = (static_cast<float>(pulses) / kPulsesPerRevolution) / seconds;
    const float rpm = rps * 60.0f;

    portENTER_CRITICAL(&state_mux);
    latest_rpm = rpm;
    if (rpm > kRunningThresholdRpm) {
      engine_run_seconds += static_cast<double>(kRpmReadMs) / 1000.0;
    }
    portEXIT_CRITICAL(&state_mux);
    return rpm;
  });
  rpm_sensor->connect_to(new SKOutputFloat("propulsion.engine.rpm"));

  auto* revolutions = new RepeatSensor<float>(kRpmReadMs, []() {
    portENTER_CRITICAL(&state_mux);
    const float rps = latest_rpm / 60.0f;
    portEXIT_CRITICAL(&state_mux);
    return rps;
  });
  revolutions->connect_to(new SKOutputFloat("propulsion.engine.revolutions"));

  auto* engine_active = new RepeatSensor<bool>(kRpmReadMs, []() {
    portENTER_CRITICAL(&state_mux);
    const bool running = latest_rpm > kRunningThresholdRpm;
    portEXIT_CRITICAL(&state_mux);
    return running;
  });
  engine_active->connect_to(new SKOutput<bool>("propulsion.engine.isRunning"));

  auto* engine_hours = new RepeatSensor<float>(kHoursReadMs, []() {
    portENTER_CRITICAL(&state_mux);
    const float hours = static_cast<float>(engine_run_seconds / kSecondsPerHour);
    portEXIT_CRITICAL(&state_mux);
    return hours;
  });
  engine_hours->connect_to(new SKOutputFloat("propulsion.engine.hours"));
}

void loop() { event_loop()->tick(); }
