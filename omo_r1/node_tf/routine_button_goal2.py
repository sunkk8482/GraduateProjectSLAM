#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time  
import RPi.GPIO as GPIO
import subprocess
import os
import signal
import pygame
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Int32


destinations = [
    Pose(Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
]
extra_destination = Pose(Point(2.0, 0.0, 0.0), Quaternion(0.0, 0.0, -0.7, 0.7))
dest_cnt=len(destinations)

BUTTON = 20
SENSOR_1 = 21 #sound detect
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

red_light = 0
blue_light = 1
blink_blue = 2

button_flag = 0

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

pygame.mixer.init()
sounds = ["msg_tts1.wav", "msg_tts2.wav", "msg_tts3.wav"]

# 0 : rotate inside 1 : button_detect 2 : reached exit
def play_sound(sound_num):
    if pygame.mixer.music.get_busy():
        return
    pygame.mixer.music.load(sounds[sound_num])
    pygame.mixer.music.play()    

def send_light_msg(light_msg):
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    pub.publish(msg)

def button_interrupt(channel):
    global button_flag
    rospy.loginfo("button presssed")
    button_flag = 1

def create_goal(destination):
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    return goal

def move_to_exit():
    global button_flag 
		rospy.loginfo(f"Fire exit start")
    send_light_msg(blue_light)
 
    #exit direction sound
    goal = create_goal(extra_destination)
    client.send_goal(goal)

    while not rospy.is_shutdown() :

				pygame.mixer.music.stop() 
        play_sound(1)

        if client.get_state() == 2 or client.get_state() == 3  :  
            rospy.loginfo("Fire exit reached")
            send_light_msg(blink_blue)
            button_flag = 0
						time.sleep(5)
            break

def move_to_destination(destination):
    global button_flag
    send_light_msg(red_light)
    goal=create_goal(destination)
    client.send_goal(goal)
    
    #rotate direction sound
    while not rospy.is_shutdown() :
        play_sound(0)
        if client.get_state() == 2 or client.get_state() == 3  :
            break
        
        elif button_flag == 1:
            move_to_exit()
            button_flag=0
            break

def main():
    try:
        rospy.loginfo("Moving path")
        pose = 0

        while not rospy.is_shutdown() :
            rospy.loginfo(f"Pose{pose} start")
            move_to_destination(destinations[pose])
						rospy.loginfo(f"Pose{pose} reached")
            pose = (pose+1) % dest_cnt

    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")
    finally:
        GPIO.cleanup()

GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_interrupt, bouncetime=1000)

if __name__ == '__main__':
    rospy.init_node('move_to_destinations_with_button')
    
    while not rospy.is_shutdown():
        read_value = GPIO.input(BUTTON)
        sound_detection = GPIO.input(SENSOR_1)
        print("Button: ", read_value)
        print("Sound1: ", sound1)

        if (sound_detection == GPIO.HIGH):
            break;
        else:
            time.sleep(0.1)

    main()