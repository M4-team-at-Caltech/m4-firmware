#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

#include "robot.hpp"
#include "utilities.hpp"

/** M4 ROBOT ARCHITECTURE -------------------------------------------------- */

M4Robot robot;

/** ROS COMMUNICATION INTERFACE -------------------------------------------- */

ros::NodeHandle ros_node_teensy;

std_msgs::Empty msg_im_alive; 
std_msgs::UInt8 msg_ack_code;

ros::Publisher pub_im_alive("/teensy/im_alive", &msg_im_alive);
ros::Publisher pub_ack_code("/teensy/ack_code", &msg_ack_code);

void msg_cb_cmd_code(const std_msgs::UInt8& cmd);
void msg_cb_robot_twist_command(const geometry_msgs::Twist& twist);

ros::Subscriber<std_msgs::UInt8> sub_cmd_code("/teensy/cmd_code", &msg_cb_cmd_code);
ros::Subscriber<geometry_msgs::Twist> sub_robot_twist_command("/robot/twist_command", &msg_cb_robot_twist_command);

/** TEENSY CODE SPECIFICS -------------------------------------------------- */

void setup() {
    ros_node_teensy.initNode();
    ros_node_teensy.advertise(pub_im_alive);

    ros_node_teensy.subscribe(sub_cmd_code);
    ros_node_teensy.subscribe(sub_robot_twist_command);
    ros_node_teensy.advertise(pub_ack_code);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {   
    robot.RoverController::test(0);
    robot.DroneController::test(0); 

    static time_t loop_time = millis();
    if(millis() - loop_time >= M4Robot::MIN_LOOP_INTERVAL_MS) {
        loop_time = millis();
        
        robot.run_controller();

        pub_im_alive.publish(&msg_im_alive);
    }

    ros_node_teensy.spinOnce();
}

/** ------------------------------------------------------------------------ */

void msg_cb_cmd_code(const std_msgs::UInt8& cmd_code) {
    msg_ack_code.data = cmd_code.data; 
    switch(cmd_code.data) {
      default:
        msg_ack_code.data = 255; //INVALID_CODE -- NOT IMPLEMENTED;
    }
    pub_ack_code.publish(&msg_ack_code);
}

void msg_cb_robot_twist_command(const geometry_msgs::Twist& twist) {
    robot.set_velocity(twist.linear.x, twist.angular.z);
    msg_ack_code.data = 13;
    pub_ack_code.publish(&msg_ack_code);
}