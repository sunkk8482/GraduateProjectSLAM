#!/usr/bin/env python

import rospy
import actionlib
import time  
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Int32
import subprocess
import os
import signal
import pygame

destinations = [
    Pose(Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
]
extra_destination = Pose(Point(2.0, 0.0, 0.0), Quaternion(0.0, 0.0, -0.7, 0.7))
dest_cnt=len(destinations)

BUTTON = 20
SENSOR_1 = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

red_light = 0
blue_light = 1
blink_blue = 2

button_flag = 0

tts_cnt = 0

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def play_sound1():
    pygame.mixer.init()
    pygame.mixer.music.load("msg_tts1.wav")
    pygame.mixer.music.play()
    

def play_sound2():
    pygame.mixer.music.load("msg_tts2.wav")
    pygame.mixer.music.play()
    

def stop_sound():
    pygame.mixer.music.stop()


def send_light_msg(light_msg):
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    pub.publish(msg)

def button_interrupt(channel):
    # 0 : red, 1 : blue, 2 : blink_blue
    global button_flag
    global tts_cnt
    rospy.loginfo("button presssed")
    button_flag = 1
    tts_cnt = 2
    rospy.sleep(0.5)

def create_goal(destination):
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    return goal

def move_to_exit():
    send_light_msg(blue_light)

    #exit direction sound

    goal = create_goal(extra_destination)
    client.send_goal(goal)
    
    while not rospy.is_shutdown() :
        if client.get_state() == 2 or client.get_state() == 3  :
            rospy.loginfo("Fire exit reached")
            break


    time.sleep(5)

def move_to_destination(destination):
    global button_flag
    global tts_cnt
    send_light_msg(red_light)

    goal=create_goal(destination)
    client.send_goal(goal)
    
    #rotate direction sound
    while not rospy.is_shutdown() :
        if tts_cnt == 1:
            play_sound1()
        elif tts_cnt == 2:
            play_sound2(msg_tts2)
            
        if client.get_state() == 2 or client.get_state() == 3  :
            stop_sound()
            break
        
        elif button_flag == 1 :
            move_to_exit()
            button_flag=0
            break

def main():
    global tts_cnt
    pygame.mixer.init()
    try:
        rospy.loginfo("Moving path")
        pose = 0

        while not rospy.is_shutdown() :
            rospy.loginfo(f"Pose{pose} start")
            tts_cnt1 = 1
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

    main()
