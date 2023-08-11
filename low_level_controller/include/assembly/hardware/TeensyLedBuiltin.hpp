#ifndef TEENSY_LED_BUILTIN
#define TEENSY_LED_BUILTIN

#include <Arduino.h>
#include <string>

class TeensyLedBuiltin {
  public:
    TeensyLedBuiltin() {
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
    }
    ~TeensyLedBuiltin() = default;

    void signal_problem(std::string msg) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
};

#endif//TEENSY_LED_BUILTIN