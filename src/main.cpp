#include <Arduino.h>
#include <Wire.h>

#include "config/EngineConfig.hpp"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensors/INA219Reader.hpp"
#include "sensors/TachRpmTracker.hpp"
#include "sensors/TemperatureSensor.hpp"

using namespace sensesp;

TemperatureSensor temp_sensor(config::kDs18b20Pin, config::kTempSensorCount,
                              config::kTempReadMs);
TachRpmTracker tach(config::kTachPin, config::kPulsesPerRevolution,
                    config::kRunningThresholdRpm);
INA219Reader oil_sensor(0x40);
INA219Reader fuel_sensor(0x41);

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.get_app();

  Wire.begin(config::kI2cSdaPin, config::kI2cSclPin);
  temp_sensor.begin();
  oil_sensor.begin();
  fuel_sensor.begin();
  tach.begin();

  auto* raw_water_temp = new RepeatSensor<float>(config::kTempReadMs, []() {
    return temp_sensor.read_kelvin(0);
  });
  raw_water_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.rawWater"));

  auto* alternator_temp = new RepeatSensor<float>(config::kTempReadMs, []() {
    return temp_sensor.read_kelvin(1);
  });
  alternator_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.alternator"));

  auto* coolant_temp = new RepeatSensor<float>(config::kTempReadMs, []() {
    return temp_sensor.read_kelvin(2);
  });
  coolant_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.coolant"));

  auto* thermostat_temp = new RepeatSensor<float>(config::kTempReadMs, []() {
    return temp_sensor.read_kelvin(3);
  });
  thermostat_temp->connect_to(
      new SKOutputFloat("propulsion.engine.temperature.thermostat"));

  auto* oil_pressure = new RepeatSensor<float>(config::kOilFuelReadMs, []() {
    const float shunt_mv = oil_sensor.read_shunt_mv();
    const float bar = INA219Reader::map_range(shunt_mv, config::kOilMinShuntMv,
                                              config::kOilMaxShuntMv, 0.0f,
                                              config::kOilMaxBar);
    return INA219Reader::clamp(bar, 0.0f, config::kOilMaxBar);
  });
  oil_pressure->connect_to(new SKOutputFloat("propulsion.engine.oilPressure"));

  auto* fuel_level = new RepeatSensor<float>(config::kOilFuelReadMs, []() {
    const float shunt_mv = fuel_sensor.read_shunt_mv();
    const float pct =
        INA219Reader::map_range(shunt_mv, config::kFuelEmptyShuntMv,
                                config::kFuelFullShuntMv, 0.0f, 1.0f);
    return INA219Reader::clamp(pct, 0.0f, 1.0f);
  });
  fuel_level->connect_to(new SKOutputFloat("tanks.fuel.level"));

  auto* rpm_sensor = new RepeatSensor<float>(config::kRpmReadMs, []() {
    return tach.sample_rpm(config::kRpmReadMs);
  });
  rpm_sensor->connect_to(new SKOutputFloat("propulsion.engine.rpm"));

  auto* revolutions = new RepeatSensor<float>(config::kRpmReadMs, []() {
    return tach.rps();
  });
  revolutions->connect_to(new SKOutputFloat("propulsion.engine.revolutions"));

  auto* engine_active = new RepeatSensor<bool>(config::kRpmReadMs, []() {
    return tach.is_running();
  });
  engine_active->connect_to(new SKOutput<bool>("propulsion.engine.isRunning"));

  auto* engine_hours = new RepeatSensor<float>(config::kHoursReadMs, []() {
    return tach.engine_hours();
  });
  engine_hours->connect_to(new SKOutputFloat("propulsion.engine.hours"));
}

void loop() { event_loop()->tick(); }
