#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <Arduino.h>

#include "controller/base_controller.hpp"

class PidController {
    float prev_measure, prev_setpoint;

    float setpoint;
    float kP, errorP, kI, errorI, kD, errorD; 
    float saturation_bound;
    float ramp_rate;

    float next_command;

  public:
    PidController(float kP, float kI, float kD, float saturation_bound=MAXFLOAT, float ramp_rate=MAXFLOAT)
    : kP{kP}, kI{kI}, kD{kD}, saturation_bound{saturation_bound}, ramp_rate{ramp_rate} { reset(); }
    ~PidController() = default;

    void update(float measure, float dt) {
        // compute respective errors, saturate integrator //todo limit ramp up
        errorP = measure - setpoint;
        errorI = constrain(errorI + errorP*dt, -saturation_bound, saturation_bound);
        errorD = ((measure - prev_measure) - (setpoint - prev_setpoint))/dt;
        // safeguard variables
        prev_measure = measure;
        prev_setpoint = setpoint;
        // output kP*e(t) + kI*∫e(t)dt + kD*de(t)/dt 
        next_command = kP*errorP + kI*errorI + kD*errorD;        
    }

    void set_target(float input) { 
        setpoint = input; 
    }
    void set_gains(float p=0, float i=0, float d=0) {
        kP = p;
        kI = i;
        kD = d;
    }
    void set_saturation(float s=MAXFLOAT) {
        saturation_bound = s;
    }
    void set_ramp(float r=MAXFLOAT) {
        ramp_rate = r;
    }

    float get_output(float pv_value) { 
        static uint32_t time = 0;
        update(pv_value, 0.001*(millis() - time));
        time = millis();
        return next_command; 
    } 
    
    void reset() {
        prev_measure = prev_setpoint = 0;
        errorP = errorI = errorD = 0;
        next_command = 0;
    }

    void init() {}
    void stop() {}
};

#endif//PID_CONTROLLER