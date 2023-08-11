#ifndef SEGWAY_CONTROLLER
#define SEGWAY_CONTROLLER

// #include "base_controller.hpp"
#include "assembly/wheel_interface.hpp"
#include "assembly/rotor_interface.hpp"

class SegwayController: public virtual WheelInterface, public virtual RotorInterface {
    // void init() override {}
    // void run() override {}
    // void stop() override {}

  public:
};

#endif//SEGWAY_CONTROLLER