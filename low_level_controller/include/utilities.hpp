#ifndef UTILITIES
#define UTILITIES

// #define DEBUG

#include <Arduino.h>
#include <math.h>

#include <vector>

#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

#define DEFAULT_BAUD 115200
#define Console Serial

enum class Sign { POS = +1, NEG = -1 };
template <typename T> T operator*(const T& operand, const Sign& value) { return operand * static_cast<T>(value); }
template <typename T> int sign(T value) { return (value >= 0) ? +1 : -1; }

using Pin = uint8_t;
using PinVector = std::vector<Pin>;

#include <Servo.h>
using PwmController = Servo;

constexpr int MAX_PWM = 2000;
constexpr int MIN_PWM = 1000;
constexpr int MID_PWM = 1500;

#endif//UTILITIES