/**
\file    capture_key.cpp
\brief   Publishes ROS topics containing the code of key strokes
\author  Gaëtan Garcia - update to ROS 2 Olivier Kermorgant
\date    30/7/2014 (ROS 2 15/10/2020)
*/
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

using namespace std;
using namespace chrono_literals;

int kbhit(void)
{

	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	return ch;
}

// this example uses old-style node, where everything is defined in the main function

int main (int argc, char** argv)
{

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("capture_key");
  auto publisher = node->create_publisher<std_msgs::msg::Int16>("/key_typed", 10);
  std_msgs::msg::Int16 message;
  rclcpp::WallRate loop_rate(100ms);
  RCLCPP_INFO(node->get_logger(), "capture_key running");

  while(rclcpp::ok())
  {
    auto key = kbhit();

    if(key > 0)
    {
      message.data = key;
      publisher->publish(message);
      rclcpp::spin_some(node);
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
