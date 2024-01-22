#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# Define the positions and orientations of the 4 destinations
destination1 = Pose(Point(0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
destination2 = Pose(Point(0.0, 0.5, 0.0), Quaternion(0.0, 0.0, 0.707, 0.707))
destination3 = Pose(Point(-0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
destination4 = Pose(Point(0.0, -0.5, 0.0), Quaternion(0.0, 0.0, -0.707, 0.707))

# List of destinations
destinations = [destination1, destination2, destination3, destination4]

def move_to_destination(destination):
    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Set up the move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination

    # Send the goal and wait for it to be achieved
    client.send_goal(goal)
    client.wait_for_result()

    # Check if the goal was achieved
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached")
    else:
        rospy.logwarn("Failed to reach goal")

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('move_to_destinations')

        # Move the robot to each destination in the list
        for destination in destinations:
            move_to_destination(destination)

    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")
