#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand


# use a global variable to store incoming messages
cmd_msg = JointCommand()

def stateCallBack(msg):
    # TODO: parse msg to build cmd_msg
    global cmd_msg


# begin main

rospy.init_node("mirror")

token = TokenHandle()

cmd_pub = rospy.Publisher(???)

# TODO prepare cmd_msg to control in position the left arm

# TODO declare a subscriber to joint states
rospy.Subscriber(???)

# TODO pick a sampling rate (in Hz)
rate = rospy.Rate(sampling rate);

# main loop, will continue until ROS is down or Ctrl-C or another node registers with the same name
while not rospy.is_shutdown():
    
    # TODO publish cmd_msg under valid conditions
    
    
    # update token, sync with sampling time and activate publish / subscribe
    token.update()
    rate.sleep()

