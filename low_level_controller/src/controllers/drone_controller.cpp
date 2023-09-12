#include "controller/drone_controller.hpp"

void DroneController::test(int id) {
    if(!id) return;
    switch(id) {
      case 1:
        // wire red led to teensy, green to output
        set_signal_source(RotorSignalSource::TEENSY);
        handover_rotor_control(); // off by default
        // fluctuate both red and green
        set_dutycycle(100, 100, 100, 100);
        delay(1000);
        set_dutycycle(50, 50, 50, 50);
        delay(1000);
        set_dutycycle(100, 100, 100, 100);
        delay(1000);
        set_dutycycle(50, 50, 50, 50);
        delay(1000);
        // red stays on, green off then comes back on for a sec
        takeover_rotor_control();
        delay(1000);
        handover_rotor_control();
        delay(1000);
        // cede signal to cube orange indicator
        set_dutycycle(100, 100, 100, 100);
        delay(250);
        set_dutycycle(0, 0, 0, 0);        
        delay(250);
        set_dutycycle(100, 100, 100, 100);
        delay(250);
        set_dutycycle(0, 0, 0, 0);        
        delay(250);
        // cede signal for 3s
        set_signal_source(RotorSignalSource::CUBE_ORANGE);
        delay(3000);
        break;
      case 2:
        set_signal_source(RotorSignalSource::TEENSY);
        set_dutycycle(10, 10, 10, 10);
        delay(1000);
      default:;
    }
}


void DroneController::init(void) {
    RotorInterface::handover_rotor_control();
}

void DroneController::update(void) {
    return; // PX4 Autopilot is in charge
}
