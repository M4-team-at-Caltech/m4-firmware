#ifndef ROVER_CONTROLLER
#define ROVER_CONTROLLER

#include "assembly/wheel_interface.hpp"

class RoverController: public virtual WheelInterface {
  public:
    void test(int id=0);

    void init(void);
    void update(float lin_vel, float ang_vel);
    void exit(void);
};

#endif//ROVER_CONTROLLER