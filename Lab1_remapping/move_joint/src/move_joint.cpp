
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float32.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>


class MoveJoint : public rclcpp::Node
{
public:
  MoveJoint() : Node("move_joint")
  {
    // node parameters
    cmd.names.push_back(declare_parameter<std::string>("joint_name", "none"));
    cmd.mode = cmd.POSITION_MODE;
    cmd.command.resize(1, 0);

    // if joint is not known, forget about this
    const std::vector<std::string> known_joints
    {
      "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2",
      "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"
    };

    if(std::find(known_joints.begin(), known_joints.end(), cmd.names[0]) == known_joints.end())
    {
      RCLCPP_INFO(get_logger(), "Parameter 'joint_name' should be a know joint: was '%s'", cmd.names[0].c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "move_joint running for %s", + cmd.names[0].c_str());

    setpoint_sub = create_subscription<example_interfaces::msg::Float32>(
                     "joint_setpoint",    // which topic
                     10,         // QoS
                     [this](example_interfaces::msg::Float32::UniquePtr msg){spCallback(msg->data);});

    // get limb to publish on the good side
    std::string out_topic = "/robot/limb/left/joint_command";
    if(cmd.names[0].find("right") != cmd.names[0].npos)
      out_topic = "/robot/limb/right/joint_command";   

    cmd_publisher = create_publisher<baxter_core_msgs::msg::JointCommand>(out_topic, 10);
  }

  inline bool validName() const
  {
    return setpoint_sub.get();
  }

private:
  // key topic
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr setpoint_sub;

  void spCallback(float sp)
  {
    if(!cmd.command.size())
      return;
    cmd.command[0] = sp;
    cmd_publisher->publish(cmd);
  }

  // joint command
  rclcpp::Publisher<baxter_core_msgs::msg::JointCommand>::SharedPtr cmd_publisher;
  baxter_core_msgs::msg::JointCommand cmd;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node{std::make_shared<MoveJoint>()};

  if(node->validName())
  {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return 0;
}
