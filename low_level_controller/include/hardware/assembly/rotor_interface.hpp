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

    void set_dutycycle(float value) { 
        Serial.print("ROTORS "); Serial.print(value); Serial.println("%");
        RotorMotorController::set_unit_signal(map(value, 0., 100., 0., 1.)); //todo fix formula&rename function 
    }
};

class RotorInterface {
    std::array<Rotor, Leg::COUNT> rotors;
    RotorSignalSelector multiplexer;

  public:
    const uint8_t CUBE_ORANGE_PWM_OUTPUT = IRRELEVANT_PIN;
    const uint8_t MOTOR_ESC_PWM_INPUT = IRRELEVANT_PIN;
    
    enum class RotorSignalSource { CUBE_ORANGE, TEENSY };

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

    void handover_rotor_control(void) {
        set_signal_source(RotorSignalSource::CUBE_ORANGE);
    }

    void takeover_rotor_control(void) {
        set_signal_source(RotorSignalSource::TEENSY);
    }

    void set_signal_source(RotorSignalSource source) {
        multiplexer.select_channel(uint8_t(source));
    }

    void set_dutycycle(uint16_t fr_pwm, uint16_t rr_pwm, uint16_t rl_pwm, uint16_t fl_pwm) {
        rotors[Leg::FR].set_dutycycle(fr_pwm);
        rotors[Leg::RR].set_dutycycle(rr_pwm);
        rotors[Leg::RL].set_dutycycle(rl_pwm);
        rotors[Leg::FL].set_dutycycle(fl_pwm);
    }

    void init() {
        takeover_rotor_control();
        for(auto& rotor : rotors) rotor.init();
    }

    void test(int id) {
        if(!id) return;
        switch(id) {
          case 1:
            set_dutycycle(0.1,0.1,0.1,0.1);
            takeover_rotor_control();
            delay(1000);
            handover_rotor_control();
            delay(3000);
            set_dutycycle(0,0,0,0);
            delay(1000);
            break;
          case 2:
            handover_rotor_control();
            break;
          case 3:
            takeover_rotor_control();
            break;
          case 4: // arm rotors through teensy
            takeover_rotor_control(); 
            Serial.println("takeover");
            set_dutycycle(0,0,0,0);
            Serial.println("0%");
            delay(3000);
            set_dutycycle(15,15,15,15);
            Serial.println("20%");
            delay(5000);
            set_dutycycle(0,0,0,0);
            Serial.println("0%");
            handover_rotor_control();
            Serial.println("handover");
            delay(2000);
            break;
        }
        while(1) {
            digitalWrite(3, 1);
            delay(500);
            digitalWrite(3, 0);
            delay(500);
            Serial.println("HERE");
        }
    }
};

#endif//ROTOR_INTERFACE