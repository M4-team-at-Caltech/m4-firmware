#ifndef WHEEL_INTERFACE
#define WHEEL_INTERFACE

#include <Arduino.h>
#include <array>

#include "hardware/component/BldcSpeedController_120F3X.hpp"
#include "hardware/component/RotaryEncoder_HEDS5500A12.hpp"
#include "hardware/assembly/definitions.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"

#include "utilities.hpp"

using WheelMotorController = BldcSpeedController_120F3X;
using WheelMotorEncoder = RotaryEncoder_HEDS5500A12;

struct Wheel : public WheelMotorController, public WheelMotorEncoder {
  private:
    // hardware specific constants
    static constexpr float gear_ratio = 6.7; // motor:wheel belt drive
    static constexpr float wheel_diameter = 0.4; // [m] //todo check value

    static constexpr float min_speed = 0.00, max_speed = 0.15; // max is 1.00 
//   protected:
public:
    // motor controller tweaks
    using WheelMotorController::set_unit_signal;
    // using WheelMotorController::set_dutycycle;
    // using WheelMotorController::brake;
    // void set_direction(bool dir) { WheelMotorController::set_direction((rotation_direction == RotDir::NEGATIVE) ? !dir : dir); }
    // rotary encoder tweaks
    using WheelMotorEncoder::write;
    using WheelMotorEncoder::read;
  public:    
    const float cycles_per_revolution = Wheel::gear_ratio * WheelMotorEncoder::grating_resolution;

    Wheel(PinVector controller_pins, RotDir rotation_direction, PinVector encoder_pins, EncDir encoding_direction, HardwareSerial& telemetry, int trim)
    : WheelMotorController{controller_pins, telemetry, true, rotation_direction, trim, max_speed}, 
      WheelMotorEncoder{encoder_pins, encoding_direction} {}
    ~Wheel(void) = default;

    // void stop_motor(void) { set_unit_signal(0); }
    // void zero_encoder(void) { WheelMotorEncoder::write(0); }

    void set_unit_speed(float uvel) {
        // brake(0);
        // set_direction((uvel > 0));
        // set_dutycycle(map(abs(uvel), 0.0, 1.0, MIN_SPEED, MAX_SPEED));
        // set_unit_signal(map(uvel, -1.0, 1.0, -max_speed, max_speed));
        set_unit_signal(uvel);
        // set_unit_signal(map(abs(uvel), 0.0, 1.0, MIN_SPEED, MAX_SPEED) * sign(uvel));
    }


    float get_angular_position(void) { 
        // returns wheel angular position since last reset
        return (2*PI) * WheelMotorEncoder::read() / cycles_per_revolution; 
    }

    void set_angular_position(float pos) { 
        WheelMotorEncoder::write(pos * cycles_per_revolution / (2*PI)); // todo rename?
    } 

    float get_angular_velocity(void) {
        // returns wheel angular velocity since last call
        static uint32_t prev_time = 0;
        static float prev_pos = 0;

        float curr_pos = WheelMotorEncoder::read(), angular_position_diff = curr_pos - prev_pos;
        uint32_t curr_time = millis(), time_diff = curr_time - prev_time;
        prev_pos = curr_pos;
        prev_time = curr_time;
        
        return angular_position_diff / time_diff;
    }
    
    void set_angular_velocity(float vel) { 
        set_unit_speed(vel); //todo implement this correctly
    }
};

class WheelInterface {
  public:
    std::array<Wheel, Leg::COUNT> wheels;

    WheelInterface(void)
    : wheels{
        Wheel{{PIN_FRONT_RIGHT_WHEEL_PWM}, RotDir::POS, {PIN_FRONT_RIGHT_WHEEL_SENSOR}, EncDir::POS, PIN_FRONT_RIGHT_WHEEL_TEL, 0},//9}, // 115
        Wheel{{PIN_REAR_RIGHT_WHEEL_PWM},  RotDir::POS, {PIN_REAR_RIGHT_WHEEL_SENSOR},  EncDir::POS, PIN_REAR_RIGHT_WHEEL_TEL,  0},//10}, // 115
        Wheel{{PIN_REAR_LEFT_WHEEL_PWM},   RotDir::NEG, {PIN_REAR_LEFT_WHEEL_SENSOR},   EncDir::NEG, PIN_REAR_LEFT_WHEEL_TEL,   0},//4}, // ref 101
        Wheel{{PIN_FRONT_LEFT_WHEEL_PWM},  RotDir::NEG, {PIN_FRONT_LEFT_WHEEL_SENSOR},  EncDir::NEG, PIN_FRONT_LEFT_WHEEL_TEL,  0},//12} // 131
      } {}
    ~WheelInterface(void) = default;

    void enable_motors(void) {
        set_unit_speed(0, 0, 0, 0);
    }

    void disable_motors(void) {
        set_unit_speed(0, 0, 0, 0);
        brake();
    }

    void set_unit_speed(float fr_uvel, float rr_uvel, float rl_uvel, float fl_uvel) {
        wheels[Leg::FR].set_unit_speed(fr_uvel);
        wheels[Leg::RR].set_unit_speed(rr_uvel);
        wheels[Leg::RL].set_unit_speed(rl_uvel);
        wheels[Leg::FL].set_unit_speed(fl_uvel);
    }
        
    void brake() {
        // for(auto& wheel : wheels) wheel.stop_motor();
        set_unit_speed(0, 0, 0, 0);
    }

    const Wheel& get_wheel(char id) const { return wheels[id]; }

    std::array<float, Leg::COUNT> get_angular_position() {
        return {
            wheels[0].get_angular_position(), 
            wheels[1].get_angular_position(), 
            wheels[2].get_angular_position(), 
            wheels[3].get_angular_position()
        };
    }
    std::array<float, Leg::COUNT> get_angular_velocity() {
        return {
            wheels[0].get_angular_velocity(), 
            wheels[1].get_angular_velocity(), 
            wheels[2].get_angular_velocity(), 
            wheels[3].get_angular_velocity()
        };
    }

    void init() {
        for(auto& wheel : wheels) wheel.init();
    }

    void test(int id, float speed=0) {
        if(!id) return;
        float s = speed; // 0.04
        switch(id) {
          case 1:
            wheels[Leg::FR].test(id); //110 100
            break;
          case 2:
            wheels[Leg::RL].set_unit_speed(s);
            delay(2000);
            wheels[Leg::RL].set_unit_speed(-s);
            delay(2000);
            wheels[Leg::RL].set_unit_speed(0);
            delay(3000);
            // wheels[Leg::FR].set_unit_speed(0.2);
            // delay(2000);
            // wheels[Leg::FR].set_unit_speed(-0.2);
            // delay(2000);
            // wheels[Leg::FR].set_unit_speed(0);
            // delay(6000);
            break;
          case 3:
            for(auto& wheel : wheels) wheel.set_unit_speed(s);
            delay(2000);
            for(auto& wheel : wheels) wheel.set_unit_speed(-s);
            delay(2000);
            // for(auto& wheel : wheels) wheel.set_unit_speed(0);
            // delay(3000);
            // for(auto& wheel : wheels) wheel.set_unit_speed(0.04);
            // delay(2000);
            // for(auto& wheel : wheels) wheel.set_unit_speed(-0.04);
            // delay(2000);
            for(auto& wheel : wheels) wheel.set_unit_speed(0);
            delay(6000);
            break;
          case 4:
            for(auto& wheel : wheels) {
                wheel.set_unit_speed(s);
                Serial.print(wheel.read());
                Serial.print(" ");
            }
            Serial.println();
            break;
          case 5:
            Serial.println("Write microseconds: ");
            delay(5000);
            float us = 1500;
            us = Serial.parseInt();
            if(us == 0) us = 1500;
            us = constrain(us, 1000, 2000);
            Serial.println(us);
            s = map(us, 1000, 2000, -1., 1.);
            Serial.println(s);
            for(auto& wheel : wheels) {
                wheel.set_unit_speed(s);
            }     
            delay(2000);       
        }
    }
};

#endif//WHEEL_INTERFACE