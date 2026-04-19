#include "sensors/INA219Reader.hpp"

INA219Reader::INA219Reader(uint8_t i2c_address) : ina219_(i2c_address) {}

void INA219Reader::begin() { ina219_.begin(); }

float INA219Reader::read_shunt_mv() { return ina219_.getShuntVoltage_mV(); }

float INA219Reader::map_range(float input, float in_min, float in_max,
                              float out_min, float out_max) {
  if (in_max == in_min) {
    return out_min;
  }
  const float t = (input - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

float INA219Reader::clamp(float value, float min_value, float max_value) {
  return value < min_value ? min_value : (value > max_value ? max_value : value);
}
