#ifndef ROVER_CONTROLLER
#define ROVER_CONTROLLER

#include "controller/base_controller.hpp"
#include "hardware/assembly/wheel_interface.hpp"

class RoverController: public BaseController, public virtual WheelInterface {
    float lin_vel, ang_vel;
  public:
    void test(int id=0);

    void init(void);
    void set_speed(float lin_vel, float ang_vel) {
        this->lin_vel = lin_vel;
        this->ang_vel = ang_vel;
    }
    void update(void);
    void stop(void);
};

#endif//ROVER_CONTROLLER