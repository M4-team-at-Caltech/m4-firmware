#ifndef SERVO_INTERFACE
#define SERVO_INTERFACE

#include <Servo.h>
#include <array>

#include "hardware/component/Led_*.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"

using ActionIndicator = LedController;

struct ServoSimulator : public ActionIndicator {
    ServoSimulator(uint8_t indicator_pin): ActionIndicator{indicator_pin} {}
    ~ServoSimulator(void) = default;
};

class ServoInterface {
  public:
    ServoSimulator servo_sim; // one indicator for 2*N_LEGS

    ServoInterface(void): servo_sim{ServoSimulator{LED_BUILTIN}} {}
    ~ServoInterface(void) = default;

    void show_activity(bool state) {
        servo_sim.set_led_state(state);
    }

    void init() {}
};

#endif//SERVO_INTERFACE