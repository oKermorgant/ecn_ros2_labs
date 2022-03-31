#ifndef IK_CLIENT_H
#define IK_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;
using baxter_core_msgs::srv::SolvePositionIK;

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync() {}

  void init(std::string name, std::string service, std::chrono::milliseconds timeout = 100ms)
  {
    node = std::make_shared<rclcpp::Node>(name);
    this->service = service;
    this->timeout = timeout;
  }

  bool call(const RequestT &req, ResponseT &res)
  {
    return call(std::make_shared<RequestT>(req), res);
  }

  bool call(const std::shared_ptr<RequestT> &req_ptr, ResponseT &res)
  {
    if(!node) return false;
    if(!client) client = node->create_client<ServiceT>(service);

    if(!client->wait_for_service(timeout))
    {
      RCLCPP_WARN(node->get_logger(), "Service %s is not reachable", service.c_str());
      // try to reconnect
      client = node->create_client<ServiceT>(service);
      if(!client->wait_for_service(timeout))
        return false;
      RCLCPP_INFO(node->get_logger(), (std::string("Reconnected to ") + service).c_str());
    }

    auto result = client->async_send_request(req_ptr);
    auto spin_result{rclcpp::spin_until_future_complete(node, result)};

    if(spin_result == rclcpp::FutureReturnCode::SUCCESS)
    {
      res = *result.get();
      return true;
    }
    return false;
  }

protected:
  std::string service;
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  std::chrono::milliseconds timeout;

};

#endif // IK_CLIENT_H