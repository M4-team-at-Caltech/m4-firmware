#ifndef ROTOR_CONTROLLER
#define ROTOR_CONTROLLER

// #include "base_controller.hpp"
#include "assembly/rotor_interface.hpp"

class DroneController: public virtual RotorInterface {
  public:
    void test(int id=0);

    void init(void);
    void update(void);
};

#endif//ROTOR_CONTROLLER