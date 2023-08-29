#ifndef SEGWAY_CONTROLLER
#define SEGWAY_CONTROLLER

#include "controller/base_controller.hpp"
#include "controller/pid_controller.hpp"
#include "hardware/assembly/wheel_interface.hpp"
#include "hardware/assembly/rotor_interface.hpp"
#include "hardware/assembly/body_interface.hpp"
// #include "estimators/state_estimator.hpp"

// DEBUG SEGWAY PARAMETERS
#define SEG_PITCH_GAIN_P    -9.0*0.99   // Pitch tracking P gain
#define SEG_PITCH_GAIN_I    -0.0   // Pitch tracking I gain
#define SEG_PITCH_GAIN_D    -0.6*0.99   // Pitch tracking D gain

#define SEG_WHEEL_GAIN_P    0.005*1.05       // wheel tracking P gain
#define SEG_WHEEL_GAIN_I    0.0*0          // wheel tracking I gain
#define SEG_WHEEL_GAIN_D    0.025*1.05    // wheel tracking D gain

#define SEG_PITCH_INT_SAT   1   // pitch error integration saturation value

#define SEGWAY_UPRIGHT_THRESHOLD  10/180.0*PI // segway starts
#define GAIN_UPRIGHT_P      -0.5
#define GAIN_UPRIGHT_I      -0.5*1.5
#define GAIN_UPRIGHT_D      0
#define UPRIGHT_SPEED       10/180.0*PI
#define UPRIGHT_MAX_THRUST  0.65           // ratio. 1 = full throttle, 0 nothing

#define UPRIGHT_SLOWDOWN_ANGLE   30/180.0*PI  // half rate and assist using wheels
#define UPRIGHT_WHEEL_ASSIST     0.4         
#define SEGWAY_TOUCHDOWN_WHEELSPEED -0.35
#define LANDING_SPEED       -7.5/180.0*PI

#define SEGWAY_LANDING_THRESHOLD 80/180.0*PI // landing angle when propellers are off


class SegwayController: public BaseController, public virtual WheelInterface, public virtual RotorInterface, public virtual BodyInterface {
  public:
    void test(int id=0); //todo

    float target_movement = 0;
    void init(void);
    void update(); //todo likely missing some end/transition conditions
    void stop(void);

    enum class SegwayState { UPRIGHT, BALANCE, SETDOWN } state = SegwayState::UPRIGHT;
    // void switch_state(SegwayState state);

    PidController rotor_controller{GAIN_UPRIGHT_P, GAIN_UPRIGHT_I, GAIN_UPRIGHT_D}; // segway upright & setdown
    PidController body_controller{SEG_PITCH_GAIN_P, SEG_PITCH_GAIN_I, SEG_PITCH_GAIN_D, SEG_PITCH_INT_SAT};  // segway balance inner loop
    PidController wheel_controller{SEG_WHEEL_GAIN_P, SEG_WHEEL_GAIN_I, SEG_WHEEL_GAIN_D}; // segway balance outer loop
};

#endif//SEGWAY_CONTROLLER