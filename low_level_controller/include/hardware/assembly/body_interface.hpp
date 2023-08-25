#ifndef BODY_INTERFACE
#define BODY_INTERFACE

#include <Arduino.h>

#include "hardware/component/Imu_BNO055.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"
#include "unit_test.hpp"

using Imu = Imu_BNO055;

struct Body : public Imu {
    Body(std::vector<uint8_t> imu_pins): Imu{imu_pins} {}
    ~Body() = default;

    void init_imu(void) {
        while(!Imu::begin()) {
            // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            delay(1000);
        }
        Imu::setAxisRemap(REMAP_CONFIG_P0);
        Imu::setAxisSign(REMAP_SIGN_P0);
    }

    using Imu::get_roll;
    using Imu::get_pitch;
    using Imu::get_yaw;
};

class BodyInterface {
  public:
    Body body;

    BodyInterface(): body{{PIN_IMU_I2C}} {}
    ~BodyInterface() = default;

    void init() {
        pinMode(LED_BUILTIN, OUTPUT);
        body.init_imu();
    }
    
    void test(UnitTest id) {
        switch(id) {
          case UnitTest::IMU:
            body.Imu::test();
            break;
          default:
            Serial.println("Invalid test request");
        }
    }
};

#endif//BODY_INTERFACE