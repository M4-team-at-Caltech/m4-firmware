#ifndef MECHATRONICS
#define MECHATRONICS

#include "hardware/assembly/body_interface.hpp"
#include "hardware/assembly/servo_interface.hpp"
#include "hardware/assembly/wheel_interface.hpp"
#include "hardware/assembly/rotor_interface.hpp"

class Mechatronics : public virtual BodyInterface, public virtual ServoInterface, public virtual WheelInterface, public virtual RotorInterface {};

#endif//MECHATRONICS