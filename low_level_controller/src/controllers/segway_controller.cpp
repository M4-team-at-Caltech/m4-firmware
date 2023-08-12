#include "controllers/segway_controller.hpp"

void SegwayController::test(int id) {
    //todo
}

void SegwayController::init(void) {
    WheelInterface::enable_motors();

    RotorInterface::set_signal_source(RotorSignalSource::TEENSY);
    RotorInterface::enable_rotors();
}

void SegwayController::update(float lin_speed, float ang_speed) {
    //todo
}