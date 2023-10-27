#include "controller/rover_controller.hpp"
#include <cmath>

#define MOTION_GAIN  0.70              // scaling for turning inputs
#define TURNING_GAIN 0.30              // scaling for turning inputs
#define MAX_SPEED 0.20                 // can take values up to 1.0 (limits the maximum speed)
#define BETA 2.0

float smooth(float x)
 {
  return 0.2*x + 0.8*pow(x,3);
 }

void RoverController::init(void) {
    WheelInterface::enable_motors();
}

void RoverController::update(void) {
    float lin_vel_smooth = smooth(lin_vel);
    float ang_vel_smooth = smooth(ang_vel);


    float cmd_r = MOTION_GAIN * lin_vel_smooth - TURNING_GAIN * ang_vel_smooth;
    float cmd_l = MOTION_GAIN * lin_vel_smooth + TURNING_GAIN * ang_vel_smooth;

    float cmd_rl =  constrain(cmd_l + 0.0,-MAX_SPEED,MAX_SPEED);
    float cmd_rr =  constrain(cmd_r + 0.04,-MAX_SPEED,MAX_SPEED);
    float cmd_fr =  constrain(cmd_r + 0.0,-MAX_SPEED,MAX_SPEED);
    float cmd_fl =  constrain(cmd_l + 0.0,-MAX_SPEED,MAX_SPEED);

    WheelInterface::set_unit_speed(-1*cmd_fr,cmd_rr,cmd_rl,cmd_fl);
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