// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>

using namespace std::chrono_literals;
using sensor_msgs::msg::JointState;
using baxter_core_msgs::msg::JointCommand;

// a useful function to get the index of a string in a vector of strings
// returns the size of the vector if not found
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
  const auto elem = std::find(names.begin(), names.end(), name);
  return std::distance(names.begin(), elem);
}

namespace lab2_mirror
{

class MirrorNode : public rclcpp::Node
{
public:
  MirrorNode(rclcpp::NodeOptions options) : Node("mirror", options)
  {
    // init whatever is needed for your node
    // these suffixes may be useful
    const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    // init command message
    cmd.mode = 1;
    for(auto name: suffixes)
    {
      // build fields .names and .command
    }

    // init subscriber
    subscriber = create_subscription<JointState>(
          "/robot/joint_states",    // which topic
          10,         // QoS
          [this](JointState::UniquePtr msg)    // callback are perfect for lambdas
    {
        mirrorToLeft(msg);
  });

    // init publishers
    publisher = create_publisher<JointCommand>("/robot/limb/left/joint_command", 10);   // topic + QoS

    // init timer - the function will be called with the given rate
    publish_timer = create_wall_timer(100ms,    // rate
                                      [&]()
    {publisher->publish(cmd);});
  }
  
private:

  std::vector<std::string> right_joints;

  // declare any subscriber / publisher / timer
  rclcpp::Subscription<JointState>::SharedPtr subscriber;

  rclcpp::Publisher<JointCommand>::SharedPtr publisher;
  JointCommand cmd;

  rclcpp::TimerBase::SharedPtr publish_timer;

  void mirrorToLeft(const JointState::UniquePtr &msg)
  {
    // this function should read msg->name for right joints and copy the value to cmd.command
    // even-numbered joints should be inverses (opposite directions to mirror)
  }
};

}



// boilerplate main function

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<lab2_mirror::MirrorNode>(options));
  rclcpp::shutdown();
  return 0;
}

