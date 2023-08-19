#include "controller/segway_controller.hpp"

void SegwayController::test(int id) {
    if(!id) return;
    switch (id) {
        //todo
      default:;
    }
}

void SegwayController::init(void) {
    WheelInterface::enable_motors();

    RotorInterface::set_signal_source(RotorSignalSource::TEENSY);
    RotorInterface::enable_rotors();
}

void SegwayController::update() {
    static float right_wheel_init_angle = 0, left_wheel_init_angle = 0;

    auto switch_state = [&](SegwayState state) { 
        rotor_controller.reset();
        body_controller.reset();
        wheel_controller.reset();
        switch(state) {
          case SegwayState::UPRIGHT:
            break;
          case SegwayState::BALANCE:
            right_wheel_init_angle = wheels[Leg::RR].get_angular_position();
            left_wheel_init_angle = wheels[Leg::RL].get_angular_position();            
            break;
          case SegwayState::SETDOWN:
            break;
        }
        this->state = state;    
    };

    float wheel_angle_measure = 0.5*((wheels[Leg::RR].get_angular_position() - right_wheel_init_angle) + (wheels[Leg::RL].get_angular_position() - left_wheel_init_angle));
    float body_pitch_measure = body.get_pitch();

    switch(state) {
      case SegwayState::UPRIGHT: {
        if(abs(body_pitch_measure) < UPRIGHT_SLOWDOWN_ANGLE) {
            rotor_controller.set_target(0.5*UPRIGHT_SPEED);
            WheelInterface::set_dutycycle(0, UPRIGHT_WHEEL_ASSIST, UPRIGHT_WHEEL_ASSIST, 0);
        } else {
            rotor_controller.set_target(UPRIGHT_SPEED);
            WheelInterface::set_dutycycle(0, 0, 0, 0);
        }
        float rotor_command = constrain(rotor_controller.get_output(body_pitch_measure), 0, UPRIGHT_MAX_THRUST);
        RotorInterface::set_dutycycle(rotor_command, 0, 0, rotor_command);

        if(abs(body_pitch_measure) > SEGWAY_UPRIGHT_THRESHOLD) {
            switch_state(SegwayState::BALANCE);
        }
      } break;
      case SegwayState::BALANCE: {
        wheel_controller.set_target(target_movement);
        float target_pitch = wheel_controller.get_output(wheel_angle_measure);
        body_controller.set_target(target_pitch);
        float motor_command = body_controller.get_output(body_pitch_measure);

        WheelInterface::set_dutycycle(0, motor_command, motor_command, 0);
        // if(abs(body_pitch_measure) > SEGWAY_UPRIGHT_THRESHOLD) switch_state(SegwayState::BALANCE);
      } break;
      case SegwayState::SETDOWN: {
        rotor_controller.set_target(LANDING_SPEED);
        float rotor_command = constrain(rotor_controller.get_output(body_pitch_measure), 0, UPRIGHT_MAX_THRUST);

        if(abs(body_pitch_measure) > SEGWAY_LANDING_THRESHOLD) {
            RotorInterface::set_dutycycle(0, 0, 0, 0);
            WheelInterface::set_dutycycle(0, 0, 0, 0);
        } else {
            RotorInterface::set_dutycycle(rotor_command, 0, 0, rotor_command);
            WheelInterface::set_dutycycle(0, SEGWAY_TOUCHDOWN_WHEELSPEED, SEGWAY_TOUCHDOWN_WHEELSPEED, 0);
        }
        // if(abs(body_pitch_measure) > SEGWAY_UPRIGHT_THRESHOLD) switch_state(SegwayState::BALANCE);
      } break;
      default:;
    }
} 

