/**
\file    move_joint_node.cpp
\brief  Joint mover using key strokes as command
 *
 *  This node reads input from the keyboard and moves a joint of the Baxter robot,
 *  incrementing/decrementing its position according to the key that was hit.
 *  The name of the joint which is controlled is set using a ROS parameter.
\author  Gaëtan Garcia
\date    30/7/2014
*/

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>

constexpr int KEY_PLUS = 43;
constexpr int KEY_MINUS = 45;
constexpr double INCR = 5*M_PI/180;

class MoveJoint : public rclcpp::Node
{
public:
  MoveJoint() : Node("move_joint")
  {
    // node parameters
    incr_key = declare_parameter<int>("incr_key", KEY_PLUS);
    decr_key = declare_parameter<int>("decr_key", KEY_MINUS);
    increment = declare_parameter<double>("increment", INCR);
    cmd.names.push_back(declare_parameter<std::string>("joint_name", "none"));
    cmd.mode = 1;

    // if joint is not known, forget about this
    const std::vector<std::string> known_joints
    {
      "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2",
      "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"
    };

    if(jointIndex(known_joints) == -1)
    {
      RCLCPP_INFO(get_logger(), "you have to specify a known joint name: was '" + cmd.names[0] + "'");
      return;
    }
    RCLCPP_INFO(get_logger(), "move_joint running for " + cmd.names[0]);

    key_subscriber = create_subscription<std_msgs::msg::Int16>(
          "/key_hit",    // which topic
          10,         // QoS
          [this](std_msgs::msg::Int16::UniquePtr msg){keyCallback(msg->data);});

    // get limb to publish on the good side
    cmd_publisher = create_publisher<baxter_core_msgs::msg::JointCommand>("/joint_command", 10);

    js_subscriber = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states",    // which topic
          10,         // QoS
          [this](sensor_msgs::msg::JointState::UniquePtr msg)    // callback are perfect for lambdas
    {
        if(auto idx = jointIndex(msg->name); idx != -1)
    {
      current_joint_value = msg->position[idx];
      cmd.command.resize(1);
    }});
}

private:
// parameters
int incr_key, decr_key;
double increment;

// key topic
rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr key_subscriber;

void keyCallback(int key)
{
  if(!cmd.command.size())
    return;
  if(key != incr_key && key != decr_key)
    return;

  if(key == incr_key)
    cmd.command[0] = current_joint_value + increment;
  else
    cmd.command[0] = current_joint_value - increment;
  cmd_publisher->publish(cmd);
}


// joint states topic
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber;
double current_joint_value = 0;

// joint command
rclcpp::Publisher<baxter_core_msgs::msg::JointCommand>::SharedPtr cmd_publisher;
baxter_core_msgs::msg::JointCommand cmd;

int jointIndex(std::vector<std::string> names)
{
  auto elem = std::find(names.begin(), names.end(), cmd.names[0]);
  if(elem == names.end())
    return -1;
  return std::distance(names.begin(), elem);
}

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveJoint>());
  rclcpp::shutdown();
  return 0;
}
