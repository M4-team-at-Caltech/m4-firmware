#ifndef BLDC_SPEED_CONTROLLER_120F3X
#define BLDC_SPEED_CONTROLLER_120F3X

#include <Arduino.h>
#include <vector>

#include "hardware/assembly/definitions.hpp"

class BldcSpeedController_120F3X {
  public:
    const uint8_t PinPwm;

    BldcSpeedController_120F3X(std::vector<uint8_t> pins)
    : PinPwm(pins[0]) {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
    }
    ~BldcSpeedController_120F3X() = default;

    void set_dutycycle(float value) {
        analogWrite(PinPwm, map(value, 0., 100., 0., 256.));
    }

};

#endif//BLDC_SPEED_CONTROLLER_120F3X