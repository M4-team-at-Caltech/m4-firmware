#ifndef ROTOR_CONTROLLER
#define ROTOR_CONTROLLER

// #include "base_controller.hpp"
#include "assembly/rotor_interface.hpp"

class DroneController: public virtual RotorInterface {
    const RotorSignalSource master = RotorSignalSource::CUBE_ORANGE;
  public:
    DroneController() {
        RotorInterface::set_signal_source(master);
        RotorInterface::disable_rotors();
    }

    // void init(void) override {    }

    void start() {
        RotorInterface::set_signal_source(master);
        RotorInterface::enable_rotors();
    }

    // void update(void) override {
    //     if(master != RotorSignalSource::TEENSY) return;
    // }
    
    void stop() {
        RotorInterface::disable_rotors();
    }
    
    void reset(void) {}    

    void test(int id=0);

    void init(void);
    void update(void);
    void exit(void);
};

#endif//ROTOR_CONTROLLER