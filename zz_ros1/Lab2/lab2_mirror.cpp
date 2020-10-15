#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <ecn_common/token_handle.h>
#include <ros/ros.h>


// use a global variable to store incoming messages
baxter_core_msgs::JointCommand cmd_msg;

void stateCallBack(const sensor_msgs::JointState &msg)
{
    // TODO: parse msg to build cmd_msg
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mirror");
    ros::NodeHandle nh;

    // related to Baxter priority queue
    ecn::TokenHandle token;

    // TODO declare a publisher to joint commands
    ros::Publisher cmd_pub = ??
    
    // TODO prepare cmd_msg to control in position the left arm

    // TODO declare a subscriber to joint states
    ros::Subscriber js_sub = ??

    // TODO pick a sampling rate (in Hz)
    ros::Rate loop(sampling rate);
    
    // main loop, will continue until ROS is down or Ctrl-C or another node registers with the same name
    while(ros::ok())
    {
        // TODO publish cmd_msg under valid conditions
        
        
        // update token, sync with sampling time and activate publish / subscribe
        token.update();
        loop.sleep();
        ros::spinOnce();
    }
}

