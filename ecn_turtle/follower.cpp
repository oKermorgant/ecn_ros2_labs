#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std::chrono_literals;
using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;
using rcl_interfaces::msg::SetParametersResult;
using rclcpp::Parameter;
using Params = std::vector<rclcpp::Parameter>;
namespace ecn_turtle
{

double toPi(double v)
{
  if(v > M_PI) return toPi(v-2*M_PI);
  if(v < -M_PI) return toPi(v+2*M_PI);
  return v;
}

class Follower : public rclcpp::Node
{
public:
  Follower(rclcpp::NodeOptions options = rclcpp::NodeOptions{}) : Node("follower", options)
  {
    // parameters
    Kv = declareBoundedParam("Kv", 2., 0., 10.);
    Kw = declareBoundedParam("Kw", 2., 0., 10.);
    d = declareBoundedParam("d", 1., 0., 5.);

    param_change = add_on_set_parameters_callback([this](const Params &change) {return parametersCallback(change);});

    // init subscribers
    pose_sub = create_subscription<Pose>(
          "/turtle2/pose",    // which topic
          10,         // QoS
          [this](Pose::UniquePtr msg)    // callback are perfect for lambdas
    {me = *msg;});

    target_sub = create_subscription<Pose>(
          "/turtle1/pose",    // which topic
          10,         // QoS
          [this](Pose::UniquePtr msg)    // callback are perfect for lambdas
    {target = *msg;});

    // init publishers
    cmd_pub = create_publisher<Twist>("/turtle2/cmd_vel", 10);   // topic + QoS

    // init timer - the function will be called with the given rate
    cmd_timer = create_wall_timer(100ms, [&](){applyCmd();});

    // communicate with turtlesim
    turtle_param_srv = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
  }
  
private:  


  // declare any subscriber / publisher / timer
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub;
  rclcpp::TimerBase::SharedPtr cmd_timer;

  rclcpp::Subscription<Pose>::SharedPtr pose_sub, target_sub;
  std::optional<Pose> me{}, target{};

  void applyCmd()
  {
    if(!me.has_value() && !target.has_value())
      return;

    const auto  dx = target->x - me->x;
    const auto  dy = target->y - me->y;

    const auto theta = toPi(atan2(dy,dx) - me->theta);

    const auto x = dx*cos(me->theta)+dy*sin(me->theta);

    auto cmd = Twist();

    cmd.linear.x = Kv*(x - d);
    cmd.angular.z = Kw*theta;
    cmd_pub->publish(cmd);

    // distance to background
    updateBackground(x - d);
  }

  // parameters
  double Kv{2.}, Kw{2.}, d{1.};
  OnSetParametersCallbackHandle::SharedPtr param_change;
  double declareBoundedParam(const std::string &name,
                           double value,
                           double low, double high)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = name;
    descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                  .set__from_value(low)
                                  .set__to_value(high)
                                 };
    return declare_parameter(name, value, descriptor);
  }

  SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    for(const auto &param: parameters)
    {
      if(param.get_name() == "Kv")  Kv = param.as_double();
      else if(param.get_name() == "Kw")  Kw = param.as_double();
      else if(param.get_name() == "d")  d = param.as_double();
    }
    return SetParametersResult().set__successful(true);
  }


  // call set parameter service
  rclcpp::AsyncParametersClient::SharedPtr turtle_param_srv;
  std::optional<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>> turtle_param_res;
  void updateBackground(double error)
  {
    if(turtle_param_res.has_value() && !turtle_param_res->valid()) return;

    // map error to RGB
    const double r = std::clamp<int>(128-error/d, 0,255);
    std::cout << "res is valid, setting r = " << r << std::endl;

    Params rgb = {Parameter("background_r", r), Parameter("background_b", 255-r)};

    turtle_param_res = turtle_param_srv->set_parameters(rgb);
  }

};

}

// register this plugin
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ecn_turtle::Follower);
