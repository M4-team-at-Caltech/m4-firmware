#include "controller/rover_controller.hpp"

#define MOTION_GAIN  1.00              // scaling for turning inputs
#define TURNING_GAIN 1.00              // scaling for turning inputs
#define WHEEL_COMMAND_DEADZONE 0.1     // deadzone around 0 speed



void RoverController::init(void) {
    WheelInterface::enable_motors();
}

void RoverController::update(void) {
    float cmd_r = constrain(MOTION_GAIN * lin_vel - TURNING_GAIN * ang_vel, -1.0, 1.0);
    float cmd_l = constrain(MOTION_GAIN * lin_vel + TURNING_GAIN * ang_vel, -1.0, 1.0);
    
    WheelInterface::set_unit_speed(cmd_r, cmd_r, cmd_l, cmd_l);
}

void RoverController::stop(void) {
    WheelInterface::disable_motors();
}




void RoverController::test(int id) {
    if(!id) return;
    switch (id) {
      case 1:
        // accelerate to some speed, direction handled by interface -- 0 fwd, 1 bwd for robot coords
        set_unit_speed(.10, .10, .10, .10); 
        delay(1000);
        // test braking
        brake(); 
        delay(1500);
        // restart at lower speed
        set_unit_speed(.05, .05, .05, .05); 
        delay(2000);
        // go faster in the other direction
        set_unit_speed(-.10, -.10, -.10, -.10); 
        delay(1000);
        // slow down again
        set_unit_speed(-.05, -.05, -.05, -.05); 
        delay(2000);
        // stop/disable wheels, wait and reenable
        brake();
        delay(1500);
        break;
      case 2:
        set_unit_speed(.05, .05, .05, .05); 
        delay(3000);   
        set_unit_speed(0, 0, 0, 0); 
        delay(3000);
        set_unit_speed(-.05, -.05, -.05, -.05);
        delay(3000);
        set_unit_speed(0, 0, 0, 0); 
        delay(3000);
        break;
      case 3:
        set_unit_speed(1, 1, 1, 1); 
        delay(3000);
        set_unit_speed(-1, -1, -1, -1); 
        delay(3000);
        break;
      default:;
    }
}