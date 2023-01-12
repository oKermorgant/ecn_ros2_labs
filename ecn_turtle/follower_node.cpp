#include "follower.cpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto follower = std::make_shared<Follower>();



  rclcpp::spin(follower);


}
