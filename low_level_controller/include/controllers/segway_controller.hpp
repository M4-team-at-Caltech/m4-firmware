#ifndef SEGWAY_CONTROLLER
#define SEGWAY_CONTROLLER

#include "assembly/wheel_interface.hpp"
#include "assembly/rotor_interface.hpp"

class SegwayController: public virtual WheelInterface, public virtual RotorInterface {
  public:
    void test(int id=0); //todo

    void init(void);
    void update(float lin_vel, float ang_vel); //todo
};

#endif//SEGWAY_CONTROLLER