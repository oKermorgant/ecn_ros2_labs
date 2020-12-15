// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>

using namespace std::chrono_literals;

// a useful function to get the index of a string in a vector of strings
inline size_t findIndex(const std::string &name, const std::vector<std::string> & names)
{
    const auto elem = std::find(names.begin(), names.end(), name);
    return std::distance(names.begin(), elem);
}


class BasicNode : public rclcpp::Node
{
public:
    BasicNode(rclcpp::NodeOptions options) : Node("node_name", options)
    {
        // init whatever is needed for your node
        
        // init subscribers
        subscriber = create_subscription<geometry_msgs::msg::Twist>(
            "topic",    // which topic
            10,         // QoS            
            [this](geometry_msgs::msg::Twist::UniquePtr msg)    // callback are perfect for lambdas
            {
                last_twist = *msg;
            });
            
        // init publishers
        publisher = create_publisher<geometry_msgs::msg::Pose2D>("ground_truth", 10);   // topic + QoS
      
        // init timer - the function will be called with the given rate
        publish_timer = create_wall_timer(100ms,    // rate
                                          [&](){duplicateArm();})
    }   
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
    geometry_msgs::msg::Twist last_twist;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher;
    geometry_msgs::msg::Pose2D pose;
    
    rclcpp::TimerBase::SharedPtr publish_timer;    
    
    
    void duplicateArm()
    {
        // use last_msg to build and publish command

        
        
    }
    
};

// register this plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BasicNode)
