#ifndef STATE_ESTIMATOR
#define STATE_ESTIMATOR

#include "hardware/assembly/wheel_interface.hpp"

class StateEstimator : public virtual WheelInterface {
  public:
    StateEstimator() = default;
    ~StateEstimator() = default;
    void update();
  private:
};

#endif//STATE_ESTIMATOR