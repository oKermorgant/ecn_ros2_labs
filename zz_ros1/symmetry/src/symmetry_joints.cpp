#include <string>

//ROS
#include <rclcpp/rclcpp.hpp>

//ROS msgs
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class SymmetryJoints : public rclcpp::Node
{
public:
    SymmetryJoints() : Node("symmetry_joints")
    {
       this->declare_parameter<std::string>("mode", "position");
       this->get_parameter("mode", mode_);
       publisher_ = this->create_publisher<baxter_core_msgs::msg::JointCommand>("/robot/limb/left/joint_command", 10);
       subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/robot/joint_states", 10, std::bind(&SymmetryJoints::topic_callback, this, _1));
    }
private:
    void topic_callback (sensor_msgs::msg::JointState::UniquePtr state_right) const
    {
        baxter_core_msgs::msg::JointCommand msg_command;
        msg_command.names = {"left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"};
              
            
        if(mode_ == "position"){
            msg_command.mode = msg_command.POSITION_MODE;
        if(state_right->name.size()==17){
                msg_command.command.clear();
              msg_command.command.push_back(-1*state_right->position[9]);
              msg_command.command.push_back(1*state_right->position[10]);
              msg_command.command.push_back(-1*state_right->position[11]);
              msg_command.command.push_back(1*state_right->position[12]);
              msg_command.command.push_back(-1*state_right->position[13]);
              msg_command.command.push_back(1*state_right->position[14]);
              msg_command.command.push_back(-1*state_right->position[15]);
    	        publisher_->publish(msg_command);//Publish the command
            }
        }        	        
        else if(mode_ == "vitesse"){
	        msg_command.mode = msg_command.VELOCITY_MODE; 
          if(state_right->name.size()==17){
		        msg_command.command.clear();
            msg_command.command.push_back(-1*state_right->velocity[9]);
            msg_command.command.push_back(1*state_right->velocity[10]);
            msg_command.command.push_back(-1*state_right->velocity[11]);
            msg_command.command.push_back(1*state_right->velocity[12]);
            msg_command.command.push_back(-1*state_right->velocity[13]);
            msg_command.command.push_back(1*state_right->velocity[14]);
            msg_command.command.push_back(-1*state_right->velocity[15]);
		        publisher_->publish(msg_command);//Publish the command
	        }
        }
    }
    
    std::string mode_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<baxter_core_msgs::msg::JointCommand>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SymmetryJoints>());
  rclcpp::shutdown();
  return 0;
}
