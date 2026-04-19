#ifndef SENSORS_INA219_READER_HPP_
#define SENSORS_INA219_READER_HPP_

#include <Adafruit_INA219.h>

class INA219Reader {
 public:
  explicit INA219Reader(uint8_t i2c_address);

  void begin();
  float read_shunt_mv();

  static float map_range(float input, float in_min, float in_max,
                         float out_min, float out_max);
  static float clamp(float value, float min_value, float max_value);

 private:
  Adafruit_INA219 ina219_;
};

#endif
