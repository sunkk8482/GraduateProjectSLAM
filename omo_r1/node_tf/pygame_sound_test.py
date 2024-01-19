#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def play_sound1():
    pygame.mixer.init()
    if pygame.mixer.music.get_busy():
        pass  
    else :
        pygame.mixer.music.load("msg_tts1.wav")
        pygame.mixer.music.play()
    
def play_sound2():
    pygame.mixer.init()
    if pygame.mixer.music.get_busy():
        pass  
    else :
        pygame.mixer.music.load("msg_tts2.wav")
        pygame.mixer.music.play()
        
def play_sound3():
    pygame.mixer.init()
    if pygame.mixer.music.get_busy():
        pass  
    else :
        pygame.mixer.music.load("msg_tts3.wav")
        pygame.mixer.music.play()
    
def stop_sound():
    pygame.mixer.music.stop()      

def send_light_msg(light_msg):
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    pub.publish(msg)

def button_interrupt(channel):
    global button_flag
    rospy.loginfo("button presssed")
    button_flag = 1
    rospy.sleep(0.5)

def create_goal(destination):
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    return goal

def move_to_exit():
    global button_flag 
    send_light_msg(blue_light) 
    #exit direction sound
    goal = create_goal(extra_destination)
    client.send_goal(goal)
    
    stop_sound()    
    while not rospy.is_shutdown() :
        play_sound2()
        if client.get_state() == 2 or client.get_state() == 3  :  
            rospy.loginfo("Fire exit reached")
            #send_light_msg(blink_blue)
            button_flag = 0
            break

    time.sleep(5)
    play_sound3()
    stop_sound()

def move_to_destination(destination):
    global button_flag
    send_light_msg(red_light)
    goal=create_goal(destination)
    client.send_goal(goal)
    
    #play_sound1()
    
    #rotate direction sound
    while not rospy.is_shutdown() :
        play_sound1()
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
    
    while True:
        read_value = GPIO.input(BUTTON)
        sound1 = GPIO.input(SENSOR_1)
        print("Button: ", read_value)
        print("Sound1: ", sound1)

        if (sound1 == GPIO.HIGH):
            print("AAAAAAAAA")
            break;
        else:
            time.sleep(0.1)

    main()
