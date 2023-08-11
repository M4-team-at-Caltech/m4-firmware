#ifndef ROTARY_ENCODER_
#define ROTARY_ENCODER_

#include <Arduino.h>
#include <vector>

class RotaryEncoder_ {
  public:
    static const uint8_t IRRELEVANT_PIN = 255;
    const uint8_t PinX;

    RotaryEncoder_(std::vector<uint8_t> pins) 
    : PinX{pins[0]} {
        for(auto& pin : pins) pinMode(pin, INPUT);
    }
    ~RotaryEncoder_() = default;
};

#endif//ROTARY_ENCODER_