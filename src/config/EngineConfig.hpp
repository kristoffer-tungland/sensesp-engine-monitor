#ifndef CONFIG_ENGINE_CONFIG_HPP_
#define CONFIG_ENGINE_CONFIG_HPP_

#include <stdint.h>

namespace config {

// Pin assignments
constexpr uint8_t kDs18b20Pin = 25;
constexpr uint8_t kTachPin = 26;
constexpr uint8_t kI2cSdaPin = 21;
constexpr uint8_t kI2cSclPin = 22;

// Temperature sensor
constexpr uint8_t kTempSensorCount = 4;
constexpr uint32_t kTempReadMs = 10000;

// INA219 sensors
constexpr uint32_t kOilFuelReadMs = 1000;
constexpr float kOilMinShuntMv = 0.0f;
constexpr float kOilMaxShuntMv = 100.0f;
constexpr float kOilMaxBar = 10.0f;
constexpr float kFuelEmptyShuntMv = 0.0f;
constexpr float kFuelFullShuntMv = 100.0f;

// Tachometer / RPM
constexpr uint32_t kRpmReadMs = 1000;
constexpr uint32_t kHoursReadMs = 5000;
constexpr float kPulsesPerRevolution = 1.0f;
constexpr float kRunningThresholdRpm = 100.0f;

// Physical constants
constexpr float kKelvinOffset = 273.15f;
constexpr float kSecondsPerHour = 3600.0f;

}  // namespace config

#endif
