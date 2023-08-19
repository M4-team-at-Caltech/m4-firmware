#ifndef WHEEL_INTERFACE
#define WHEEL_INTERFACE

#include <Arduino.h>
#include <array>

#include "hardware/component/BldcSpeedController_X002IFRCSJ.hpp"
#include "hardware/component/RotaryEncoder_HEDS5500A12.hpp"
#include "hardware/assembly/definitions.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"

using WheelMotorController = BldcSpeedController_X002IFRCSJ;
using WheelMotorEncoder = RotaryEncoder_HEDS5500A12;

struct Wheel : public WheelMotorController, public WheelMotorEncoder {
    Wheel(std::vector<uint8_t> controller_pins, std::vector<uint8_t> encoder_pins, bool reversed_rotation, bool reversed_encoding)
    : WheelMotorController{controller_pins}, WheelMotorEncoder{encoder_pins, gear_ratio},
      reversed_rotation{reversed_rotation}, reversed_encoding{reversed_encoding} {}
    ~Wheel(void) = default;

    using WheelMotorController::set_dutycycle;
    void set_direction(bool direction) { WheelMotorController::set_direction(reversed_rotation ? !direction : direction); }
    using WheelMotorController::brake;

    int32_t read(void) { return WheelMotorEncoder::read() * (reversed_encoding ? -1 : 1); }
    using WheelMotorEncoder::write;

    float get_angular_position(void) { 
        // returns wheel angular position since last reset (write 0)
        return 2*PI * WheelMotorEncoder::read() / cycles_per_revolution; 
    }

  private: 
    static constexpr float gear_ratio = 6.7; // motor:wheel belt drive
    static constexpr float wheel_diameter = 0.4; // [m] //todo check value

    const bool reversed_rotation;
    const bool reversed_encoding;

    const float cycles_per_revolution = Wheel::gear_ratio * WheelMotorEncoder::grating_resolution;
};

class WheelInterface {
  public:
    std::array<Wheel, Leg::COUNT> wheels;

    WheelInterface(void)
    : wheels{
        Wheel{{PIN_FRONT_RIGHT_WHEEL_MOTOR_ESC}, {PIN_FRONT_RIGHT_WHEEL_SENSOR }, 1, 1},
        Wheel{{PIN_REAR_RIGHT_WHEEL_MOTOR_ESC}, {PIN_REAR_RIGHT_WHEEL_SENSOR}, 1, 1},
        Wheel{{PIN_REAR_LEFT_WHEEL_MOTOR_ESC}, {PIN_REAR_LEFT_WHEEL_SENSOR }, 0, 0},
        Wheel{{PIN_FRONT_LEFT_WHEEL_MOTOR_ESC}, {PIN_FRONT_LEFT_WHEEL_SENSOR }, 0, 0}
      } {}
    ~WheelInterface(void) {
        // wheels[FR].detach();
        // wheels[RR].detach();
        // wheels[RL].detach();
        // wheels[FL].detach();
    }

    void enable_motors(void) {
        // wheels[FR].stop(1);
        // wheels[RR].stop(1);
        // wheels[RL].stop(1);
        // wheels[FL].stop(1);
    }

    void disable_motors(void) {
        set_unit_speed(0, 0, 0, 0);
        // wheels[FR].stop(0);
        // wheels[RR].stop(0);
        // wheels[RL].stop(0);
        // wheels[FL].stop(0);    
    }

    void set_dutycycle(uint16_t fr_pwm, uint16_t rr_pwm, uint16_t rl_pwm, uint16_t fl_pwm) {
        wheels[Leg::FR].set_dutycycle(fr_pwm);
        wheels[Leg::RR].set_dutycycle(rr_pwm);
        wheels[Leg::RL].set_dutycycle(rl_pwm);
        wheels[Leg::FL].set_dutycycle(fl_pwm);
    }

    void set_direction(uint16_t fr_dir, uint16_t rr_dir, uint16_t rl_dir, uint16_t fl_dir) {
        wheels[Leg::FR].set_direction(fr_dir);
        wheels[Leg::RR].set_direction(rr_dir);
        wheels[Leg::RL].set_direction(rl_dir);
        wheels[Leg::FL].set_direction(fl_dir);
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

    Wheel& get_wheel(char id) { return wheels[id]; }

    // std::array<int32_t, Leg::COUNT> get_position(void) {
    //     return {
    //         wheels[Leg::FR].get_position(),
    //         wheels[Leg::RR].get_position(),
    //         wheels[Leg::RL].get_position(),
    //         wheels[Leg::FL].get_position()        
    //     };
    // }
};

#endif//WHEEL_INTERFACE