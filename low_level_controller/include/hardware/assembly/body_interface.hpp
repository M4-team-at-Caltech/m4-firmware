#ifndef BODY_INTERFACE
#define BODY_INTERFACE

#include <Arduino.h>

#include "hardware/component/Imu_BNO055.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"

using Imu = Imu_BNO055;

struct Body : public Imu {
    Body(std::vector<uint8_t> imu_pins): Imu{imu_pins} {}
    ~Body() = default;

    float get_pitch(void) { return Imu::getVector(Adafruit_BNO055::VECTOR_EULER).y(); }
};

class BodyInterface {
  public:
    Body body;

    BodyInterface(): body{{PIN_IMU}} {}
    ~BodyInterface() = default;
};

#endif//BODY_INTERFACE