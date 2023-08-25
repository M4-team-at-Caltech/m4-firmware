#ifndef JOINT_CONTROLLER
#define JOINT_CONTROLLER

#include "controller/rover_controller.hpp"
#include "controller/drone_controller.hpp"
#include "controller/segway_controller.hpp"

class JointController : public RoverController, public DroneController, public SegwayController {
  public:
    enum class Mobility { ROVER, DRONE, SEGWAY };

};

#endif//JOINT_CONTROLLER