#include "estimator/state_estimator.hpp"

void StateEstimator::update(void) {
    // get robot speed
    WheelInterface::get_angular_velocity();
}