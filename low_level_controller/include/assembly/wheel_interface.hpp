#ifndef WHEEL_INTERFACE
#define WHEEL_INTERFACE

#include <Servo.h>
#include <array>

#include "hardware/SpeedControllerBLDC_X002IFRCSJ.hpp"
#include "hardware/RotaryEncoder_.hpp"
#include "definitions.hpp"

using WheelMotorController = SpeedControllerBLDC_X002IFRCSJ;
using WheelMotorEncoder = RotaryEncoder_;

struct Wheel : public WheelMotorController, public WheelMotorEncoder {
    Wheel(std::vector<uint8_t> controller_pins, std::vector<uint8_t> encoder_pins, bool reversed_rotation)
    : WheelMotorController{controller_pins}, WheelMotorEncoder{encoder_pins},
      reversed_rotation{reversed_rotation} {
        // for(auto& pin : controller_pins) pinMode(pin, OUTPUT);
        // for(auto& pin : encoder_pins) pinMode(pin, INPUT);
    }
    ~Wheel(void) = default;

    void set_dutycycle(uint16_t value) { 
        WheelMotorController::set_dutycycle(value); //todo fix formula&rename function 
    }
    
    void set_direction(bool direction) { 
        WheelMotorController::set_direction(reversed_rotation ? direction : !direction);
    }

    using WheelMotorController::brake;
    using WheelMotorController::stop;

    // using WheelMotorEncoder::get_position; //todo implement
    // using WheelMotorEncoder::get_speed; //todo implement

  private: 
    bool reversed_rotation;
};

class WheelInterface {
    std::array<Wheel, N_LEGS> wheels;

    void init()  {}
    void run()  {}
    void stop()  {}

  public:
    WheelInterface(void)
    : wheels{
        Wheel{{PIN_FRONT_RIGHT_WHEEL_MOTOR_ESC}, {PIN_FRONT_RIGHT_WHEEL_SENSOR }, 1},
        Wheel{{PIN_REAR_RIGHT_WHEEL_MOTOR_ESC}, {PIN_REAR_RIGHT_WHEEL_SENSOR}, 1},
        Wheel{{PIN_REAR_LEFT_WHEEL_MOTOR_ESC}, {PIN_REAR_LEFT_WHEEL_SENSOR }, 0},
        Wheel{{PIN_FRONT_LEFT_WHEEL_MOTOR_ESC}, {PIN_FRONT_LEFT_WHEEL_SENSOR }, 0}
      } {}
    ~WheelInterface(void) {
        // wheels[FR].detach();
        // wheels[RR].detach();
        // wheels[RL].detach();
        // wheels[FL].detach();
    }

    void enable_rotors(void) {
        for(auto& wheel : wheels) wheel.stop(1);
        // wheels[FR].stop(1);
        // wheels[RR].stop(1);
        // wheels[RL].stop(1);
        // wheels[FL].stop(1);
    }

    void disable_rotors(void) {
        for(auto& wheel : wheels) wheel.stop(0);
        // wheels[FR].stop(0);
        // wheels[RR].stop(0);
        // wheels[RL].stop(0);
        // wheels[FL].stop(0);    
    }

    void set_dutycycle(uint16_t fr_pwm, uint16_t rr_pwm, uint16_t rl_pwm, uint16_t fl_pwm) {
        wheels[FR].set_dutycycle(fr_pwm);
        wheels[RR].set_dutycycle(rr_pwm);
        wheels[RL].set_dutycycle(rl_pwm);
        wheels[FL].set_dutycycle(fl_pwm);
    }

    void set_direction(uint16_t fr_dir, uint16_t rr_dir, uint16_t rl_dir, uint16_t fl_dir) {
        wheels[FR].set_direction(fr_dir);
        wheels[RR].set_direction(rr_dir);
        wheels[RL].set_direction(rl_dir);
        wheels[FL].set_direction(fl_dir);
    }
    void set_unit_speed(float fr_uvel, float rr_uvel, float rl_uvel, float fl_uvel) {
        static auto dir = [](float uvel) { return uvel > 0; };
        static auto dty = [](float uvel) { return constrain(abs(uvel) * 100., 0., 100.); };

        set_dutycycle(dty(fr_uvel), dty(rr_uvel), dty(rl_uvel), dty(fl_uvel));
        set_direction(dir(fr_uvel), dir(rr_uvel), dir(rl_uvel), dir(fl_uvel));
    }

    void brake(bool state) {
        for(auto& wheel : wheels) wheel.brake(state);
    }

};

#endif//WHEEL_INTERFACE