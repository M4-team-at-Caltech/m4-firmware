#ifndef TEENSY_PIN_ALLOCATION
#define TEENSY_PIN_ALLOCATION

#include "hardware/assembly/definitions.hpp"

#define IRRELEVANT_PIN UINT8_MAX

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

/** COMMUNICATIONS --------------------------------------------------------- */
#define COM_USB_ROS SerialUSB // Serial0

/** ROTORS ----------------------------------------------------------------- */
    /* MULTIPLEXER */
#define PIN_ROTOR_CHANNEL_SELECT    4

#define PIN_FRONT_RIGHT_ROTOR_PWM   3 // Motor 1 CCW
#define PIN_REAR_RIGHT_ROTOR_PWM    28 // Motor 4 CW
#define PIN_REAR_LEFT_ROTOR_PWM     2 // Motor 2 CCW
#define PIN_FRONT_LEFT_ROTOR_PWM    29 // Motor 3 CW

#define PIN_ROTOR_GROUND_SIGNALS    IRRELEVANT_PIN 

/** WHEELS ----------------------------------------------------------------- */
    /* MOTOR CONTROLLERS */
#define PIN_FRONT_RIGHT_WHEEL_PWM   22
#define PIN_FRONT_RIGHT_WHEEL_TEL   Serial4
#define PIN_REAR_RIGHT_WHEEL_PWM    14
#define PIN_REAR_RIGHT_WHEEL_TEL    Serial8
#define PIN_REAR_LEFT_WHEEL_PWM     23
#define PIN_REAR_LEFT_WHEEL_TEL     Serial5
#define PIN_FRONT_LEFT_WHEEL_PWM    15
#define PIN_FRONT_LEFT_WHEEL_TEL    Serial6

/** SERVOS ----------------------------------------------------------------- */

/** SENSORS ---------------------------------------------------------------- */
    /* ENCODERS */
#define PIN_FRONT_RIGHT_ENCODER_CHA 10
#define PIN_FRONT_RIGHT_ENCODER_CHB 9
#define PIN_REAR_RIGHT_ENCODER_CHA  12
#define PIN_REAR_RIGHT_ENCODER_CHB  11
#define PIN_REAR_LEFT_ENCODER_CHA   6
#define PIN_REAR_LEFT_ENCODER_CHB   5
#define PIN_FRONT_LEFT_ENCODER_CHA  8
#define PIN_FRONT_LEFT_ENCODER_CHB  7

#define PIN_FRONT_RIGHT_WHEEL_SENSOR    PIN_FRONT_RIGHT_ENCODER_CHA, PIN_FRONT_RIGHT_ENCODER_CHB
#define PIN_REAR_RIGHT_WHEEL_SENSOR     PIN_REAR_RIGHT_ENCODER_CHA,  PIN_REAR_RIGHT_ENCODER_CHB
#define PIN_REAR_LEFT_WHEEL_SENSOR      PIN_REAR_LEFT_ENCODER_CHA,   PIN_REAR_LEFT_ENCODER_CHB
#define PIN_FRONT_LEFT_WHEEL_SENSOR     PIN_FRONT_LEFT_ENCODER_CHA,  PIN_FRONT_LEFT_ENCODER_CHB
    /* IMU */
#define PIN_IMU_SDA 18 
#define PIN_IMU_SCL 19
#define PIN_IMU_I2C PIN_IMU_SDA, PIN_IMU_SCL // I2C0


#endif//TEENSY_PIN_ALLOCATION