#include <Arduino.h>

#include "robot.hpp"

auto M4Robot::transform(Mobility mode) -> uint8_t {
    // select_channel(mode != Mobility::DRONE ? 0 : 1);
    this->mode = mode;
    return 0;
}

auto M4Robot::ready_controller() -> uint8_t{
    //TODO set this shit up, stop everything, toggle signals etc.
    switch(mode) {
      case Mobility::ROVER:
        RoverController::init();
        break;
      case Mobility::DRONE:
        DroneController::init();
        break;
      default:;
    }
    return 0;
}

auto M4Robot::set_velocity(float lin_vel, float ang_vel) -> uint8_t {
    this->lin_vel = lin_vel;
    this->ang_vel = ang_vel;
    return 0;
}

void M4Robot::run_controller(void) {
    switch(mode) {
      case Mobility::ROVER:
        RoverController::update(lin_vel, ang_vel);
        break;
      case Mobility::DRONE:
        DroneController::update();
        break;
      default:;
    }
}

