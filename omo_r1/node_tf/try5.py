import rospy
import actionlib
import time  
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Int32
import subprocess
import os

rospy.init_node('move_to_destinations_with_button')

msg = Int32()
pub = rospy.Publisher('my_topic', Int32, queue_size=10)
msg.data = 0
pub.publish(msg)
rospy.sleep(0.5)