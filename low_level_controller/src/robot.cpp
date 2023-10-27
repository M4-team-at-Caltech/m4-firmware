#include <Arduino.h>

#include "robot.hpp"

void M4Robot::init_hardware(void) {    
    // BodyInterface::init();
    RotorInterface::init();
    WheelInterface::init();
    
    attachInterrupt(PIN_REBOOT, _reboot_Teensyduino_, RISING); 
}


void M4Robot::transform(Mobility mode) {
    stop_operations();
    this->mode = mode;
}

void M4Robot::init_controller(void) {
    assign_pilot(Authority::COMMANDER);

    switch(mode) {
      case Mobility::ROVER:
        // transform(Mobility::ROVER);
        RoverController::init();
        break;
      case Mobility::DRONE:
        // transform(Mobility::DRONE);
        DroneController::init();
        break;
      case Mobility::SEGWAY:
        // transform(Mobility::SEGWAY);
        SegwayController::init();
        break;
    }
}

auto M4Robot::set_pose(float px, float py, float pz, float qx, float qy, float qz, float qw) -> uint8_t {
    //todo
    return 1;
}

auto M4Robot::set_velocity(float lin_vel, float ang_vel) -> uint8_t {
    // this->lin_vel = lin_vel;
    // this->ang_vel = ang_vel;
    RoverController::set_speed(lin_vel, ang_vel);
    return 0;
}

void M4Robot::run_controller(void) {
    if(pilot == Authority::NONE) return;

    switch(mode) {
      case Mobility::ROVER:
        RoverController::update();
        break;
      case Mobility::DRONE:
        DroneController::update();
        break;
      case Mobility::SEGWAY:
        SegwayController::update();
        break;
    }
}

void M4Robot::update_state(void) {
    StateEstimator::update();
}

void M4Robot::assign_pilot(Authority pilot) {
    this->pilot = pilot;
}

void M4Robot::stop_operations(void) {
    switch(mode) {
      case Mobility::ROVER:
        RoverController::stop();
        break;
      case Mobility::DRONE:
        DroneController::stop();
        break;
      case Mobility::SEGWAY:
        SegwayController::stop();
        break;
      default:;
        WheelInterface::set_unit_speed(0, 0, 0, 0);
    }

    assign_pilot(Authority::NONE);
}

auto M4Robot::execute_command(uint8_t command) -> uint8_t {
    switch(Command(command)) {
      case Command::TRANSFORM_ROVER:
        transform(M4Robot::Mobility::ROVER);
        break;
      case Command::TRANSFORM_DRONE:
        transform(M4Robot::Mobility::DRONE);
        break;
      case Command::TRANSFORM_SEGWAY:
        transform(M4Robot::Mobility::SEGWAY);
        break;
      case Command::DRIVE:
      case Command::FLY:
      case Command::BALANCE:
        init_controller();
        break;
      case Command::WHEEL_IN:{
        // float t = 0.1;
        WheelInterface::set_unit_speed(0.13, 0.13, 0.13, -0.13);
        // WheelInterface::set_unit_speed(t, t, t, -t);
        break;}
      case Command::WHEEL_OUT:{
        WheelInterface::set_unit_speed(-0.1, -0.1,-0.13,0.1);

        // WheelInterface::set_unit_speed(-s, -s, -s, s);
        break;}
      case Command::STOP:
        stop_operations();
        break;
      default:
        command = uint8_t(Command::NOT_IMPLEMENTED);
        break;
    }
    return command; //todo send back ack only when command done, so wait for segway setdown
}

void M4Robot::test(UnitTest id) {
    switch(id) {
      case UnitTest::BODY:
        Serial.print("\nX "); Serial.print(body.get_roll());
        Serial.print("\tY "); Serial.print(body.get_pitch());
        Serial.print("\tZ "); Serial.print(body.get_yaw());
        delay(50);
        break;
      case UnitTest::IMU: 
        BodyInterface::test(UnitTest::IMU);
        break;
      case UnitTest::WHEELS:
        WheelInterface::set_unit_speed(0.1, 0, 0, 0);
        delay(2000);
        WheelInterface::set_unit_speed(0, 0.1, 0, 0);
        delay(2000);
        WheelInterface::set_unit_speed(0, 0, 0.1, 0);
        delay(2000);
        WheelInterface::set_unit_speed(0, 0, 0, 0.1);
        delay(2000);
      default:;
    }
}

