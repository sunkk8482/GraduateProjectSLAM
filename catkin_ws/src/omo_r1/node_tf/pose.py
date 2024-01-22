#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('set_initial_pose')

# Create a publisher to send the initial pose
pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

# Wait for the '2D Pose Estimate' tool to be available in RViz
rospy.loginfo('Waiting for 2D Pose Estimate tool...')
rospy.wait_for_service('/global_localization')

# Publish the initial pose
initial_pose = PoseWithCovarianceStamped()
initial_pose.header.frame_id = 'map'
initial_pose.pose.pose.position.x = <initial_x> # set initial x position here
initial_pose.pose.pose.position.y = <initial_y> # set initial y position here
initial_pose.pose.pose.orientation.z = <initial_z> # set initial orientation here
initial_pose.pose.covariance[0] = 0.25  # set the covariance of x
initial_pose.pose.covariance[7] = 0.25  # set the covariance of y
initial_pose.pose.covariance[35] = 0.06853891945200942  # set the covariance of orientation

pose_pub.publish(initial_pose)
rospy.loginfo('Initial pose set.')