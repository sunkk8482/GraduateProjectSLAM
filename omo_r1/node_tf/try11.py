#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

def set_initial_pose(pose_msg):
    """
    Set initial pose of Cartographer based on 2D pose estimate from RViz
    """
    rospy.set_param('/cartographer/initial_pose_x', pose_msg.pose.pose.position.x)
    rospy.set_param('/cartographer/initial_pose_y', pose_msg.pose.pose.position.y)
    rospy.set_param('/cartographer/initial_pose_z', pose_msg.pose.pose.position.z)

    # Convert quaternion to yaw angle
    qx = pose_msg.pose.pose.orientation.x
    qy = pose_msg.pose.pose.orientation.y
    qz = pose_msg.pose.pose.orientation.z
    qw = pose_msg.pose.pose.orientation.w
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    rospy.set_param('/cartographer/initial_pose_yaw', yaw)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('set_cartographer_initial_pose_from_rviz')

    # Subscribe to 2D pose estimate topic
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, set_initial_pose)

    # Spin ROS node
    rospy.spin()
