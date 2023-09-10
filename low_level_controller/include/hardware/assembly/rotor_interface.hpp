#ifndef ROTOR_INTERFACE
#define ROTOR_INTERFACE

#include <Arduino.h>
#include <array>

#include "hardware/component/MultiplexerQuad2To1_CD74AC157E.hpp"
#include "hardware/component/BldcSpeedController_120F3X.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"

using RotorMotorController = BldcSpeedController_120F3X;
using RotorSignalSelector = MultiplexerQuad2To1_CD74AC157E;

struct Rotor : public RotorMotorController {
    Rotor(std::vector<uint8_t> controller_pins, RotDir rotation_direction)
    : RotorMotorController{controller_pins, false, rotation_direction} {}
    ~Rotor() = default;

    void set_dutycycle(uint16_t value) { 
        RotorMotorController::set_unit_signal(value); //todo fix formula&rename function 
    }
};

class RotorInterface {
    std::array<Rotor, Leg::COUNT> rotors;
    RotorSignalSelector multiplexer;

  public:
    const uint8_t CUBE_ORANGE_PWM_OUTPUT = IRRELEVANT_PIN;
    const uint8_t MOTOR_ESC_PWM_INPUT = IRRELEVANT_PIN;
    
    enum class RotorSignalSource { TEENSY, CUBE_ORANGE };

    RotorInterface(void)
    : rotors{ 
        Rotor{{PIN_FRONT_RIGHT_ROTOR_PWM}, RotDir::POS}, 
        Rotor{{PIN_REAR_RIGHT_ROTOR_PWM},  RotDir::POS}, 
        Rotor{{PIN_REAR_LEFT_ROTOR_PWM},   RotDir::POS},
        Rotor{{PIN_FRONT_LEFT_ROTOR_PWM},  RotDir::POS} 
      },
      multiplexer{{
        PIN_ROTOR_CHANNEL_SELECT,
        PIN_FRONT_RIGHT_ROTOR_PWM, CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT, //todo check this initalization
        PIN_REAR_RIGHT_ROTOR_PWM,  CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        PIN_REAR_LEFT_ROTOR_PWM,   CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        PIN_FRONT_LEFT_ROTOR_PWM,  CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        PIN_ROTOR_GROUND_SIGNALS
      }} {}
    ~RotorInterface(void) {
        // for(auto& rotor : rotors) rotor.detach();
        // rotors[FR].detach();
        // rotors[RR].detach();
        // rotors[RL].detach();
        // rotors[FL].detach();
    }

    void enable_rotors(void) {
        // multiplexer.ground_signals(0);
        set_signal_source(RotorSignalSource::CUBE_ORANGE);
    }

    void disable_rotors(void) {
        // multiplexer.ground_signals(1);
        set_signal_source(RotorSignalSource::TEENSY);
    }

    void set_signal_source(RotorSignalSource source) {
        multiplexer.select_channel(uint8_t(source));
    }

    void set_dutycycle(uint16_t fr_pwm, uint16_t rr_pwm, uint16_t rl_pwm, uint16_t fl_pwm) {
        // rotors[FR].writeMicroseconds(fr_pwm);
        // rotors[RR].writeMicroseconds(rr_pwm);
        // rotors[RL].writeMicroseconds(rl_pwm);
        // rotors[FL].writeMicroseconds(fl_pwm);
        rotors[Leg::FR].set_dutycycle(fr_pwm);
        rotors[Leg::RR].set_dutycycle(rr_pwm);
        rotors[Leg::RL].set_dutycycle(rl_pwm);
        rotors[Leg::FL].set_dutycycle(fl_pwm);
    }

    void init() {}

    void test(int id) {
        if(!id) return;
        switch(id) {
          case 1:
            set_dutycycle(0.1,0.1,0.1,0.1);
            disable_rotors();
            delay(1000);
            enable_rotors();
            delay(3000);
            set_dutycycle(0,0,0,0);
            delay(1000);
            break;
          case 2:
            enable_rotors();
            break;
          case 3:
            disable_rotors();
            break;
        }
    }
};

#endif//ROTOR_INTERFACE