#ifndef SPEED_CONTROLLER_BLDC_
#define SPEED_CONTROLLER_BLDC_

#include <Arduino.h>
#include <vector>

class SpeedControllerBLDC_ {
  public:
    static const uint8_t IRRELEVANT_PIN = 255;
    const uint8_t PinPwm;

    SpeedControllerBLDC_(std::vector<uint8_t> pins)
    : PinPwm(pins[0]) {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
    }
    ~SpeedControllerBLDC_() = default;

    void set_dutycycle(float value) {
        analogWrite(PinPwm, map(value, 0., 100., 0., 256.));
    }

};

#endif//SPEED_CONTROLLER_BLDC_