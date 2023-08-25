#ifndef TEENSY_PIN_ALLOCATION
#define TEENSY_PIN_ALLOCATION

#include "hardware/assembly/definitions.hpp"

/** UTILITIES ============================================================== */
    /* SHORTHAND */
#define PIN_FR(NAME) PIN_FRONT_RIGHT_##NAME
#define PIN_RR(NAME) PIN_REAR_RIGHT_##NAME
#define PIN_RL(NAME) PIN_REAR_LEFT_##NAME
#define PIN_FL(NAME) PIN_FRONT_LEFT_##NAME

/** TEENSY PIN ALLOCATION ================================================== */
/** SPECIAL ---------------------------------------------------------------- */
#define PIN_REBOOT  0
#define PIN_LED     13 // LED_BUILTIN

/** ROTORS ----------------------------------------------------------------- */
    /* MULTIPLEXER */
#define PIN_ROTOR_CHANNEL_SELECT  4

#define PIN_FRONT_RIGHT_ROTOR_PWM 3 // Motor 1 CCW
#define PIN_REAR_RIGHT_ROTOR_PWM  28 // Motor 4 CW
#define PIN_REAR_LEFT_ROTOR_PWM   2 // Motor 2 CCW
#define PIN_FRONT_LEFT_ROTOR_PWM  29 // Motor 3 CW

#define PIN_ROTOR_GROUND_SIGNALS  IRRELEVANT_PIN 

/** WHEELS ----------------------------------------------------------------- */
    /* MOTOR CONTROLLERS */
#define PIN_FRONT_RIGHT_WHEEL_PWM   A9
#define PIN_FRONT_RIGHT_WHEEL_DIR   39
#define PIN_FRONT_RIGHT_WHEEL_BREAK 40
#define PIN_FRONT_RIGHT_WHEEL_STOP  IRRELEVANT_PIN
#define PIN_FRONT_RIGHT_WHEEL_MOTOR_ESC \
    PIN_FR(WHEEL_PWM), PIN_FR(WHEEL_DIR), PIN_FR(WHEEL_BREAK), PIN_FR(WHEEL_STOP)
#define PIN_REAR_RIGHT_WHEEL_PWM   A8
#define PIN_REAR_RIGHT_WHEEL_DIR   37
#define PIN_REAR_RIGHT_WHEEL_BREAK 38
#define PIN_REAR_RIGHT_WHEEL_STOP  IRRELEVANT_PIN
#define PIN_REAR_RIGHT_WHEEL_MOTOR_ESC \
    PIN_RR(WHEEL_PWM), PIN_RR(WHEEL_DIR), PIN_RR(WHEEL_BREAK), PIN_RR(WHEEL_STOP)
#define PIN_REAR_LEFT_WHEEL_PWM   A1
#define PIN_REAR_LEFT_WHEEL_DIR   35
#define PIN_REAR_LEFT_WHEEL_BREAK 36
#define PIN_REAR_LEFT_WHEEL_STOP  IRRELEVANT_PIN
#define PIN_REAR_LEFT_WHEEL_MOTOR_ESC \
    PIN_RL(WHEEL_PWM), PIN_RL(WHEEL_DIR), PIN_RL(WHEEL_BREAK), PIN_RL(WHEEL_STOP)
#define PIN_FRONT_LEFT_WHEEL_PWM   A0
#define PIN_FRONT_LEFT_WHEEL_DIR   33
#define PIN_FRONT_LEFT_WHEEL_BREAK 34
#define PIN_FRONT_LEFT_WHEEL_STOP  IRRELEVANT_PIN
#define PIN_FRONT_LEFT_WHEEL_MOTOR_ESC \
    PIN_FL(WHEEL_PWM), PIN_FL(WHEEL_DIR), PIN_FL(WHEEL_BREAK), PIN_FL(WHEEL_STOP)

/** SERVOS ----------------------------------------------------------------- */

/** SENSORS ---------------------------------------------------------------- */
    /* ENCODERS */
#define PIN_FRONT_RIGHT_ENCODER_CHA 6
#define PIN_FRONT_RIGHT_ENCODER_CHB 5
#define PIN_FRONT_RIGHT_WHEEL_SENSOR \
    PIN_FR(ENCODER_CHA), PIN_FR(ENCODER_CHB)
#define PIN_REAR_RIGHT_ENCODER_CHA 8
#define PIN_REAR_RIGHT_ENCODER_CHB 7
#define PIN_REAR_RIGHT_WHEEL_SENSOR \
    PIN_RR(ENCODER_CHA), PIN_RR(ENCODER_CHB)
#define PIN_REAR_LEFT_ENCODER_CHA 10
#define PIN_REAR_LEFT_ENCODER_CHB 9
#define PIN_REAR_LEFT_WHEEL_SENSOR \
    PIN_RL(ENCODER_CHA), PIN_RL(ENCODER_CHB)
#define PIN_FRONT_LEFT_ENCODER_CHA 12
#define PIN_FRONT_LEFT_ENCODER_CHB 11
#define PIN_FRONT_LEFT_WHEEL_SENSOR \
    PIN_FL(ENCODER_CHA), PIN_FL(ENCODER_CHB)
    /* IMU */
#define PIN_IMU_SDA 18 // SDA
#define PIN_IMU_SCL 19 // SCL
#define PIN_IMU_I2C PIN_IMU_SDA, PIN_IMU_SCL // I2C0

/** COMMUNICATIONS --------------------------------------------------------- */
#define COM_USB_ROS SerialUSB // Serial0

#endif//TEENSY_PIN_ALLOCATION