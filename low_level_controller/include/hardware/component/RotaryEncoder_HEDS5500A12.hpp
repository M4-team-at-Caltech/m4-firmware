#ifndef ROTARY_ENCODER_HEDS5500A12
#define ROTARY_ENCODER_HEDS5500A12

#include <Arduino.h>
#include <Encoder.h>
#include <vector>

#include "hardware/assembly/definitions.hpp"

class RotaryEncoder_HEDS5500A12 : public Encoder {
  public:
    const uint8_t PinChA;
    const uint8_t PinChB;

    static constexpr uint32_t grating_resolution = 500; // [cycles/revolution]

    RotaryEncoder_HEDS5500A12(std::vector<uint8_t> pins, float gear_ratio) 
    : Encoder{pins[0], pins[1]}, PinChA{pins[0]}, PinChB{pins[1]} {}
    ~RotaryEncoder_HEDS5500A12() = default;
  
    using Encoder::read;
    using Encoder::write;
};

#endif//ROTARY_ENCODER_HEDS5500A12