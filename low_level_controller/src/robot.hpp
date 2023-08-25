#ifndef M4_ROBOT
#define M4_ROBOT

#include <Arduino.h>

#include "hardware/mechatronics.hpp"
#include "estimator/state_estimator.hpp"
#include "controller/joint_controller.hpp"

class M4Robot : public Mechatronics, public StateEstimator, public JointController {

    float px, py, pz, qx, qy, qz, qw;
    float lin_vel, ang_vel;
  public:
    enum class Authority { NONE, COMMANDER };

    static const uint32_t MIN_LOOP_INTERVAL_MS = 20;

    M4Robot(): pilot{Authority::NONE}, mode{Mobility::ROVER} {}
    ~M4Robot() = default;

    void init_hardware(void);
    void assign_pilot(Authority pilot);
    void transform(Mobility mode);
    void init_controller(void);
    void run_controller(void);
    void stop_operations(void);

    auto set_pose(float px, float py, float pz, float qx, float qy, float qz, float qw) -> uint8_t; //todo
    auto set_velocity(float lin_vel, float ang_vel) -> uint8_t;

    void test(UnitTest id);

   private:
    Authority pilot;
    Mobility mode;
};

#endif//M4_ROBOT