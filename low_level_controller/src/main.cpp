#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "robot.hpp"
#include "commands.hpp"
// #include "hardware/wiring/definitions.hpp"

/** M4 ROBOT ARCHITECTURE -------------------------------------------------- */

M4Robot robot;

/** ROS COMMUNICATION INTERFACE -------------------------------------------- */

ros::NodeHandle ros_node_teensy;

std_msgs::Empty msg_im_alive; 
std_msgs::UInt8 msg_ack_code;

ros::Publisher pub_im_alive("/teensy/im_alive", &msg_im_alive);
ros::Publisher pub_ack_code("/teensy/ack_code", &msg_ack_code);

void msg_cb_cmd_code(const std_msgs::UInt8& cmd);
ros::Subscriber<std_msgs::UInt8> sub_cmd_code("/teensy/cmd_code", &msg_cb_cmd_code);


void msg_cb_robot_pose_estimate(const geometry_msgs::Pose& pose);
void msg_cb_robot_twist_command(const geometry_msgs::Twist& twist);
ros::Subscriber<geometry_msgs::Pose> sub_robot_pose_estimate("/robot/pose_estimate", &msg_cb_robot_pose_estimate);
ros::Subscriber<geometry_msgs::Twist> sub_robot_twist_command("/robot/twist_command", &msg_cb_robot_twist_command);

/** TEENSY CODE SPECIFICS -------------------------------------------------- */
#include "hardware/component/Imu_BNO055.hpp"
#define DEBUG
Imu_BNO055 imutest{{18, 19}};

void setup() {
#ifdef DEBUG
#else
    ros_node_teensy.initNode();
    ros_node_teensy.advertise(pub_im_alive);

    ros_node_teensy.subscribe(sub_cmd_code);
    ros_node_teensy.advertise(pub_ack_code);

    ros_node_teensy.subscribe(sub_robot_pose_estimate);
    ros_node_teensy.subscribe(sub_robot_twist_command);

    pinMode(LED_BUILTIN, OUTPUT);

    attachInterrupt(REBOOT_PIN, _reboot_Teensyduino_, RISING); 
#endif//DEBUG
}

void loop() { 
#ifdef DEBUG
    imutest.test();
#else
    robot.RoverController::test(0);
    robot.DroneController::test(0); 

    static time_t loop_time = millis();
    if(millis() - loop_time >= M4Robot::MIN_LOOP_INTERVAL_MS) {
        loop_time = millis();
        
        robot.run_controller();

        pub_im_alive.publish(&msg_im_alive);
    }
    ros_node_teensy.spinOnce();
#endif//DEBUG
}

/** ------------------------------------------------------------------------ */

void msg_cb_cmd_code(const std_msgs::UInt8& cmd_code) {
    msg_ack_code.data = cmd_code.data; 
    switch(Command(cmd_code.data)) {
      case Command::TRANSFORM_ROVER:
        robot.transform(M4Robot::Mobility::ROVER);
        break;
      case Command::TRANSFORM_DRONE:
        robot.transform(M4Robot::Mobility::DRONE);
        break;
      case Command::TRANSFORM_SEGWAY:
        robot.transform(M4Robot::Mobility::SEGWAY);
        break;
      case Command::DRIVE: 
      case Command::FLY:
      case Command::BALANCE:
        robot.init_controller();
        break;
      case Command::STOP:
        robot.stop_operations();
        break;
      default:
        msg_ack_code.data = uint8_t(Command::NOT_IMPLEMENTED);
    }
    pub_ack_code.publish(&msg_ack_code);
}

/** ------------------------------------------------------------------------ */

void msg_cb_robot_pose_estimate(const geometry_msgs::Pose& pose) {
    if(!robot.set_pose(
        pose.position.x, pose.position.y, pose.position.z, 
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    )) {
        msg_ack_code.data = uint8_t(Command::POSE_RECEIVED);
        pub_ack_code.publish(&msg_ack_code);
    }
}

void msg_cb_robot_twist_command(const geometry_msgs::Twist& twist) {
    if(!robot.set_velocity(twist.linear.x, twist.angular.z)) {
        msg_ack_code.data = uint8_t(Command::TWIST_ACK);
        pub_ack_code.publish(&msg_ack_code);
    }
}