#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 1)
        self.goal = None

        self.navigator = BasicNavigator()

        self.timer = self.create_timer(5., self.replan)




if __name__ == '__main__':
    rclpy.init()
    rclpy.spin(Navigator())




    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 7.0
    goal_pose.pose.position.y = 0.5
    goal_pose.pose.orientation.w = 1.0

    # Get the path, smooth it
    path = navigator.getPath(initial_pose, goal_pose)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)
