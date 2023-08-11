#ifndef ROTOR_INTERFACE
#define ROTOR_INTERFACE

#include <Servo.h>
#include <array>

#include "hardware/MultiplexerQuad2To1_CD74AC157E.hpp"
#include "hardware/SpeedControllerBLDC_.hpp"
#include "definitions.hpp"

using RotorMotorController = SpeedControllerBLDC_;
using RotorSignalSelector = MultiplexerQuad2To1_CD74AC157E;

struct Rotor : public RotorMotorController {

    Rotor(std::vector<uint8_t> controller_pins)
    : RotorMotorController{controller_pins} {
        // for(auto& pin : controller_pins) pinMode(pin, OUTPUT);  
    }
    ~Rotor() = default;

    void set_dutycycle(uint16_t value) { 
        RotorMotorController::set_dutycycle(value); //todo fix formula&rename function 
    }
};

class RotorInterface {
    std::array<Rotor, N_LEGS> rotors;
    RotorSignalSelector multiplexer;

  public:
    const uint8_t CUBE_ORANGE_PWM_OUTPUT = RotorSignalSelector::IRRELEVANT_PIN;
    const uint8_t MOTOR_ESC_PWM_INPUT = RotorSignalSelector::IRRELEVANT_PIN;
    
    enum class RotorSignalSource { TEENSY, CUBE_ORANGE };

    RotorInterface(void)
    : rotors{ 
        Rotor{{PIN_FRONT_RIGHT_ROTOR_PWM}}, 
        Rotor{{PIN_REAR_RIGHT_ROTOR_PWM}}, 
        Rotor{{PIN_REAR_LEFT_ROTOR_PWM}},
        Rotor{{PIN_FRONT_LEFT_ROTOR_PWM}} 
      },
      multiplexer{{
        PIN_ROTOR_CHANNEL_SELECT,
        rotors[FR].PinPwm, CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        rotors[FR].PinPwm, CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        rotors[FR].PinPwm, CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
        rotors[FR].PinPwm, CUBE_ORANGE_PWM_OUTPUT, MOTOR_ESC_PWM_INPUT,
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
        multiplexer.ground_signals(0);
    }

    void disable_rotors(void) {
        multiplexer.ground_signals(1);    
    }

    void set_dutycycle(uint16_t fr_pwm, uint16_t rr_pwm, uint16_t rl_pwm, uint16_t fl_pwm) {
        // rotors[FR].writeMicroseconds(fr_pwm);
        // rotors[RR].writeMicroseconds(rr_pwm);
        // rotors[RL].writeMicroseconds(rl_pwm);
        // rotors[FL].writeMicroseconds(fl_pwm);
        rotors[FR].set_dutycycle(fr_pwm);
        rotors[RR].set_dutycycle(rr_pwm);
        rotors[RL].set_dutycycle(rl_pwm);
        rotors[FL].set_dutycycle(fl_pwm);
    }

    void set_signal_source(RotorSignalSource source) {
        multiplexer.select_channel(uint8_t(source));
    }
};

#endif//ROTOR_INTERFACE