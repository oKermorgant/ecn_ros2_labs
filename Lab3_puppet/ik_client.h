#ifndef IK_CLIENT_H
#define IK_CLIENT_H

#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_client.h>

using namespace std::chrono_literals;
using baxter_core_msgs::srv::SolvePositionIK;

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync(std::string name): node(std::make_shared<rclcpp::Node>(name))
  {    }

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT &req)
  {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  ResponseT sendRequest(const std::shared_ptr<RequestT> &req_ptr)
  {
    auto result = client->async_send_request(req_ptr);
    rclcpp::spin_until_future_complete(node, result);
    return *result.get();
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};

#endif // IK_CLIENT_H
