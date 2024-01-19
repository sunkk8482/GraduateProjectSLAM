#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('send_goal')

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
goal.target_pose.header.stamp = rospy.Time.now()


goal.target_pose.pose.position.x = -1.68394994736
goal.target_pose.pose.position.y = -1.23710918427
goal.target_pose.pose.position.z = 0.0


goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.985356028573
goal.target_pose.pose.orientation.w = 0.170509521599



pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
msg = MoveBaseActionGoal()
msg.header = goal.target_pose.header
msg.goal_id.stamp = rospy.Time.now()
msg.goal_id.id = ''
msg.goal = goal


pub.publish(msg)