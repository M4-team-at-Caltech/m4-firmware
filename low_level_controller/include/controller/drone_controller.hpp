#ifndef ROTOR_CONTROLLER
#define ROTOR_CONTROLLER

#include "controller/base_controller.hpp"
#include "hardware/assembly/rotor_interface.hpp"

class DroneController: public BaseController, public virtual RotorInterface {
  public:
    void test(int id=0);

    void init(void);
    void update(void);
    void stop(void) {}
};

#endif//ROTOR_CONTROLLER