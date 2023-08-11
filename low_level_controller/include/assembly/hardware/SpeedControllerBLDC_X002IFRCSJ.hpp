#ifndef SPEED_CONTROLLER_BLDC_X002IFRCSJ
#define SPEED_CONTROLLER_BLDC_X002IFRCSJ

#include <Arduino.h>
#include <vector>

class SpeedControllerBLDC_X002IFRCSJ {
  public:
    static const uint8_t IRRELEVANT_PIN = 255;
    const uint8_t PinPwm;
    const uint8_t PinDir;
    const uint8_t PinBreak;
    const uint8_t PinStop;

    SpeedControllerBLDC_X002IFRCSJ(std::vector<uint8_t> pins)
    : PinPwm{pins[0]}, PinDir{pins[1]}, PinBreak{pins[2]}, PinStop{pins[3]} {
        for(auto& pin : pins) pinMode(pin, OUTPUT);
    }
    ~SpeedControllerBLDC_X002IFRCSJ() = default;

    void set_dutycycle(float value) {
        analogWrite(PinPwm, map(value, 0., 100., 0., 256.));
    }

    void set_direction(bool direction) {
        // inverses rotation direction
        digitalWrite(PinDir, direction);
    }

    void brake(bool state) {
        // de/activate breaking system - active high
        digitalWrite(PinBreak, state);
    }

    void stop(bool state) {
        // en/disable phase switching - active low
        digitalWrite(PinStop, state);
    }
};

#endif//SPEED_CONTROLLER_BLDC_X002IFRCSJ