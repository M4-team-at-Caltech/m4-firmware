#include "controllers/drone_controller.hpp"

void DroneController::test(int id) {
    if(!id) return;
    switch(id) {
      case 1:
        // wire red led to teensy, green to output
        set_signal_source(RotorSignalSource::TEENSY);
        enable_rotors(); // off by default
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
        disable_rotors();
        delay(1000);
        enable_rotors();
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
      default:;
    }
}


void DroneController::init(void) {
    RotorInterface::set_signal_source(RotorSignalSource::CUBE_ORANGE);
    RotorInterface::enable_rotors();
}

void DroneController::update(void) {
    return; // PX4 Autopilot is in charge
}
