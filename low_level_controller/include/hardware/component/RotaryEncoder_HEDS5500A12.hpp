#ifndef ROTARY_ENCODER_HEDS5500A12
#define ROTARY_ENCODER_HEDS5500A12

#include <Arduino.h>
#include <Encoder.h>
#include <vector>

#include "hardware/assembly/definitions.hpp"
#include "utilities.hpp"

using EncDir = Sign;

class RotaryEncoder_HEDS5500A12 : public Encoder {
  public:
    const uint8_t PinChA;
    const uint8_t PinChB;

    static constexpr uint32_t grating_resolution = 500; // [cycles/revolution]

    RotaryEncoder_HEDS5500A12(std::vector<uint8_t> pins, EncDir direction) 
    : Encoder{pins[0], pins[1]}, PinChA{pins[0]}, PinChB{pins[1]}, direction{direction} {}
    ~RotaryEncoder_HEDS5500A12() = default;
  
    int32_t read(void) { return Encoder::read() * direction; }
    void write(int32_t p) { Encoder::write(p * direction); }
    
   private:
    const EncDir direction;
};

#endif//ROTARY_ENCODER_HEDS5500A12