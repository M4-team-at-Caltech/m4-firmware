#include <Arduino.h>

#include "robot.hpp"

void M4Robot::transform(Mobility mode) {
    stop_operations();
    this->mode = mode;
}

void M4Robot::init_controller(void) {
    assign_pilot(Authority::COMMANDER);

    switch(mode) {
      case Mobility::ROVER:
        RoverController::init();
        break;
      case Mobility::DRONE:
        DroneController::init();
        break;
      case Mobility::SEGWAY:
        SegwayController::init();
        break;
    }
}

auto M4Robot::set_pose(float px, float py, float pz, float qx, float qy, float qz, float qw) -> uint8_t {
    //todo
    return 1;
}

auto M4Robot::set_velocity(float lin_vel, float ang_vel) -> uint8_t {
    this->lin_vel = lin_vel;
    this->ang_vel = ang_vel;
    return 0;
}

void M4Robot::run_controller(void) {
    if(pilot == Authority::NONE) return;

    switch(mode) {
      case Mobility::ROVER:
        RoverController::update(lin_vel, ang_vel);
        break;
      case Mobility::DRONE:
        DroneController::update();
        break;
      case Mobility::SEGWAY:
        SegwayController::update(lin_vel, ang_vel);
        break;
    }
}

void M4Robot::assign_pilot(Authority pilot) {
    this->pilot = pilot;
}

void M4Robot::stop_operations(void) {
    WheelInterface::disable_motors();
    RotorInterface::disable_rotors();

    assign_pilot(Authority::NONE);
}
