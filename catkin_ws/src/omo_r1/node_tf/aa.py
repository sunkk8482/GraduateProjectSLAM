#!/usr/bin/env python

import rospy
import actionlib
import time  
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Int32


def send_light_msg(light_msg):
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    pub.publish(msg)
    time.sleep(1)


rospy.init_node('led')

#while not rospy.is_shutdown():
for i in range(2):
    send_light_msg(2)
    time.sleep(5)
