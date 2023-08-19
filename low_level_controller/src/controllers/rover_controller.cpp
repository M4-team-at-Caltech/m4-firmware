#include "controller/rover_controller.hpp"

#define TURNING_GAIN 0.75              // scaling for turning inputs
#define WHEEL_COMMAND_DEADZONE 0.1     // deadzone around 0 speed

void RoverController::test(int id) {
    if(!id) return;
    switch (id) {
      case 1:
        // accelerate to some speed, direction handled by interface -- 0 fwd, 1 bwd for robot coords
        set_dutycycle(50, 50, 50, 50); 
        set_direction(0, 0, 0, 0);
        delay(1000);
        // test braking
        brake(1); 
        delay(1500);
        brake(0);
        // restart at lower speed
        set_dutycycle(25, 25, 25, 25); 
        delay(2000);
        // go faster in the other direction
        set_dutycycle(50, 50, 50, 50); 
        set_direction(1, 1, 1, 1);    
        delay(1000);
        // slow down again
        set_dutycycle(25, 25, 25, 25); 
        delay(2000);
        // stop/disable wheels, wait and reenable
        brake(1);
        delay(1500);
        brake(0);
        break;
      case 2:
        set_dutycycle(50, 50, 50, 50); 
        set_direction(1, 1, 1, 1);    
        delay(3000);
        set_dutycycle(50, 50, 50, 50); 
        set_direction(0, 0, 0, 0);    
        delay(3000);
        break;
      default:;
    }
}


void RoverController::init(void) {
    WheelInterface::enable_motors();
}

void RoverController::update(void) {
    float cmd_r = lin_vel - TURNING_GAIN * ang_vel;
    float cmd_l = lin_vel + TURNING_GAIN * ang_vel;
    
    set_unit_speed(cmd_r, cmd_r, cmd_l, cmd_l);
}


