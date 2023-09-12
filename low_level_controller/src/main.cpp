#include <Arduino.h>
#include <ros.h>

#include "robot.hpp"

/** ROS COMMUNICATION INTERFACE -------------------------------------------- */

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

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

/** M4 ROBOT ARCHITECTURE -------------------------------------------------- */

M4Robot robot{ros_node_teensy};
// #define DEBUG

// #include <std_msgs/Int32MultiArray.h>
// #include <array>
// std_msgs::Int32MultiArray array_msg;
// ros::Publisher array_pub("array_topic", &array_msg);

/** TEENSY CODE ------------------------------------------------------------ */

void setup() {
#ifdef DEBUG
    Serial.begin(9600);
    Serial.setTimeout(1500);
    Serial.println("Setup");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
    robot.init_hardware();
    delay(5000);
    Serial.println("Loop");
#else
    ros_node_teensy.initNode();
    ros_node_teensy.advertise(pub_im_alive);

    ros_node_teensy.subscribe(sub_cmd_code);
    ros_node_teensy.advertise(pub_ack_code);

    ros_node_teensy.subscribe(sub_robot_pose_estimate);
    ros_node_teensy.subscribe(sub_robot_twist_command);

    robot.init_hardware();

    // ros_node_teensy.advertise(array_pub);
#endif//DEBUG
}

void loop() { 
#ifdef DEBUG
#include "unit_test.hpp"
    static time_t loop_time = millis();
    if(millis() - loop_time >= 2 * M4Robot::MIN_LOOP_INTERVAL_MS) {
        loop_time = millis();
        // robot.RoverController::test(2);
        // robot.WheelInterface::test(4, 0.3);
        robot.RotorInterface::test(2);
        // Serial.println(map(0.4, 0., 30., 0, 400));
        // robot.DroneController::test(2);
        // robot.BodyInterface::test(UnitTest::IMU);
        // Serial.println("here");
    }
#else
    static time_t loop_time = millis();
    if(millis() - loop_time >= M4Robot::MIN_LOOP_INTERVAL_MS) {
        loop_time = millis();
        robot.update_state();
        robot.run_controller();
        pub_im_alive.publish(&msg_im_alive);

        // std::array<int, 4> a = robot.WheelInterface::read_enc();
        // for(int i = 0; i < 4; i++) {
        //     array_msg.data[i] = a[i];
        // }
        // array_pub.publish(&array_msg);    
        
        ros_node_teensy.spinOnce();
    }
#endif//DEBUG
}

/** ------------------------------------------------------------------------ */

void msg_cb_cmd_code(const std_msgs::UInt8& cmd_code) {
    msg_ack_code.data = robot.execute_command(cmd_code.data);
    pub_ack_code.publish(&msg_ack_code);
}

/** ------------------------------------------------------------------------ */

void msg_cb_robot_pose_estimate(const geometry_msgs::Pose& pose) {
    if(!robot.set_pose( //todo refactor this for all notion of commands to be in robot
        pose.position.x, pose.position.y, pose.position.z, 
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    )) {
        msg_ack_code.data = uint8_t(Command::POSE_RECEIVED);
        pub_ack_code.publish(&msg_ack_code);
    }
}

void msg_cb_robot_twist_command(const geometry_msgs::Twist& twist) {
    if(!robot.set_velocity(twist.linear.x, twist.angular.z)) { //todo refactor this for all notion of commands to be in robot
        msg_ack_code.data = uint8_t(Command::TWIST_ACK);
        pub_ack_code.publish(&msg_ack_code);
    }
}