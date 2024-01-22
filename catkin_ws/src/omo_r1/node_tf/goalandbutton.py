#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

BUTTON = 20

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def button_callback(channel):
    rospy.loginfo("button")
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

    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()

    if result:
        rospy.loginfo('Goal reached')
    else:
        rospy.loginfo('Failed to reach goal')


GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_callback, bouncetime=300)

if __name__ == '__main__':
    try:
        rospy.init_node('move_robot')
        rospy.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()