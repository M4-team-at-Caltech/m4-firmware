#ifndef LED_CONTROLLER
#define LED_CONTROLLER

#include <Arduino.h>
#include <string>

#include "hardware/assembly/definitions.hpp"

class LedController {
  public:
    const uint8_t PinLed;    
    
    LedController(uint8_t pin): PinLed{pin} {
        pinMode(pin, OUTPUT);
    }
    ~LedController() = default;

    void set_led_state(bool state) {
        digitalWrite(PinLed, state);
    }

    void set_led_dutycycle(uint32_t value) {
        analogWrite(PinLed, value);
    }

    void toggle_led(void) {
        digitalWrite(PinLed, !digitalRead(PinLed));
    }

    void blink_led(uint32_t msec) {
        return blink_led(msec, msec);
    }
    void blink_led(uint32_t msec_h, uint32_t msec_l, uint32_t n_times=1) {
        while(n_times--) {
            set_led_state(HIGH);
            delay(msec_h);
            set_led_state(LOW);
            delay(msec_l);
        }
    }
};

#endif//LED_CONTROLLER
