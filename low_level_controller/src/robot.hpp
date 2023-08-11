#ifndef M4_ROBOT
#define M4_ROBOT

#include <Arduino.h>

#include "controllers/rover_controller.hpp"
#include "controllers/drone_controller.hpp"
#include "controllers/segway_controller.hpp"


class M4Robot : public RoverController, public DroneController, public SegwayController {
    float lin_vel, ang_vel;
  public:
    enum class Authority { HUMAN, COMPUTER } pilot; // for future use
    enum class Mobility { ROVER, DRONE, SEGWAY } mode;

    static const uint32_t MIN_LOOP_INTERVAL_MS = 20;

    M4Robot(): mode{Mobility::ROVER} {}
    ~M4Robot() = default;

    auto transform(Mobility mode) -> uint8_t;
    auto ready_controller(void) -> uint8_t;
    void run_controller(void);

    auto set_velocity(float lin_vel, float ang_vel) -> uint8_t;
// void init() override final {}
// void run() override final {}
// void stop() override final {}
   private:
};

#endif//M4_ROBOT