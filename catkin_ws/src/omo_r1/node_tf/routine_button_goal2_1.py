#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int32
import pygame

# Define destinations
destinations = [
    Pose(Point(1.6, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    Pose(Point(1.6, 1.8, 0.0), Quaternion(0.0, 0.0, 0.7, 0.7)),
    Pose(Point(0.0, 1.7, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)),
    Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
]
extra_destination = Pose(Point(1.7, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

# GPIO setup
BUTTON = 20
SENSOR_1 = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Light signals
RED_LIGHT = 1
BLUE_LIGHT = 2
BLINK_BLUE = 3

# Global flags
button_flag = 0
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Sound functions
def play_sound(file):
    pygame.mixer.init()
    if not pygame.mixer.music.get_busy():
        pygame.mixer.music.load(file)
        pygame.mixer.music.play()

def stop_sound():
    pygame.mixer.music.stop()

# Light message function
def send_light_msg(light_msg):
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    for _ in range(2):
        pub.publish(msg)

# Button interrupt handler
def button_interrupt(channel):
    global button_flag
    rospy.loginfo("Button pressed")
    button_flag = 1
    rospy.sleep(0.5)

# Create move goal
def create_goal(destination):
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    return goal

# Move to exit
def move_to_exit():
    global button_flag
    send_light_msg(BLUE_LIGHT)
    
    goal = create_goal(extra_destination)
    client.send_goal(goal)
    
    stop_sound()
    while not rospy.is_shutdown():
        play_sound("msg_tts2.wav")
        if client.get_state() in [2, 3]:
            rospy.loginfo("Fire exit reached")
            send_light_msg(BLINK_BLUE)
            stop_sound()
            play_sound("msg_tts3.wav")
            time.sleep(5)
            button_flag = 0
            break

# Move to destination
def move_to_destination(destination):
    global button_flag
    send_light_msg(RED_LIGHT)
    goal = create_goal(destination)
    client.send_goal(goal)
    
    while not rospy.is_shutdown():
        play_sound("msg_tts1.wav")
        if client.get_state() in [2, 3]:
            break
        elif button_flag == 1:
            move_to_exit()
            button_flag = 0
            break

# Main function
def main():
    try:
        rospy.loginfo("Moving path")
        pose = 0

        while not rospy.is_shutdown():
            rospy.loginfo(f"Pose {pose} start")
            move_to_destination(destinations[pose])
            rospy.loginfo(f"Pose {pose} reached")
            pose = (pose + 1) % len(destinations)

    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")
    finally:
        GPIO.cleanup()

# Setup button interrupt
GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_interrupt, bouncetime=1000)

if __name__ == '__main__':
    rospy.init_node('move_to_destinations_with_button')
    
    for _ in range(2):
        send_light_msg(RED_LIGHT)
        
    main()

