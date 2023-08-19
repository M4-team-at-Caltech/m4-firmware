#ifndef GENERAL_INTERFACE
#define GENERAL_INTERFACE

#include "hardware/component/Imu_BNO055.hpp"
#include "hardware/assembly/rotor_interface.hpp"
#include "hardware/assembly/servo_interface.hpp"
#include "hardware/assembly/wheel_interface.hpp"
#include "hardware/assembly/teensy_pin_allocation.hpp"


using Imu = Imu_BNO055;

class GeneralInterface 
: public virtual RotorInterface, 
  public virtual ServoInterface, 
  public virtual WheelInterface { 
    Imu imu;
};

#endif//GENERAL_INTERFACE