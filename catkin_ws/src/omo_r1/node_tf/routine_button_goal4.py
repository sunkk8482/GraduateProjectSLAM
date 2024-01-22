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

current_path = os.path.dirname(os.path.realpath(__file__))
filename1 = os.path.join(current_path, "msg_tts1.wav")
filename2 = os.path.join(current_path, "msg_tts2.wav")


# Define the positions and orientations of the 4 destinations
destinations = [
    Pose(Point(3.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    Pose(Point(3.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.7, 0.7)),
    Pose(Point(3.0, 0.0, 0.0), Quaternion(0.0, 0.0, -0.7, 0.7)),
    Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
]
extra_destination = Pose(Point(3.0, -1.0, 0.0), Quaternion(0.0, 0.0, -0.7, 0.7)) #the position of out 
dest_cnt=len(destinations)

#Button definition
BUTTON = 20
SENSOR_1 = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

red_light=0
blue_light=1
blink_blue=2

flag=0

tts_cnt=0

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def send_light_msg(light_msg) : # send a msg to arduino for light
    # 0 : red, 1 : blue, 2 : blink_blue
    rospy.loginfo(f"light_msg = {light_msg}")
    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = light_msg
    pub.publish(msg)
    rospy.sleep(0.5)

def move_to_destination(destination):
    # Create an action client for the move_base action server
    client.wait_for_server()

    # Set up the move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    
    # Send the goal and wait for it to be achieved
    client.send_goal(goal)
    
    while not rospy.is_shutdown() :
        #rospy.loginfo(client.get_state())
        if client.get_state() == 2 or client.get_state() == 3  :
            break

def button_callback(channel):
    global flag, blue_light, blink_blue, filename2
    
    flag=1
    
    rospy.loginfo("Button pressed")
    subprocess.call(["aplay", filename2])

    send_light_msg(blue_light) # light on color of fire exit 
    move_to_destination(extra_destination)
    rospy.loginfo("Fire exit reached")
    send_light_msg(blink_blue) # light on color of fire exit 
    

def main():
    try:
        global flag, red_light, filename1 tts_cnt
        
        rospy.loginfo("Moving path")
        tts_cnt = 1
        if tts_cnt = 1:
            while(True):
                subprocess.call(["aplay", filename1])
                time.sleep(1)
            

        pose = 0
        while not rospy.is_shutdown() :
            
            rospy.loginfo("move call")
            send_light_msg(red_light) # light on color of moving path
            
            if flag==1 :
                time.sleep(5) # 5 second
                
            move_to_destination(destinations[pose])
            rospy.loginfo(f"Pose{pose} reached")
            pose = (pose+1) % dest_cnt # loop path 0~4
            flag=0
            #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")
    finally:
        GPIO.cleanup()

#Button interrupt
GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_callback, bouncetime=1000)

#if __name__ == '__main__':
rospy.init_node('move_to_destinations_with_button')

try:
    while True:
        read_value = GPIO.input(BUTTON)
        sound1 = GPIO.input(SENSOR_1)
        print("Button: ", read_value)
        print("Sound1: ", sound1)

        if (sound1 == GPIO.HIGH):
            # rosrun 명령을 실행하여 Python 파일을 실행합니다.


            #GPIO.output(Relaypin, GPIO.HIGH)
            # rosrun 명령을 실행하여 Python 파일을 실행합니다.
            print("AAAAAAAAA")
            main()
        else:
            time.sleep(0.1)

finally:
    # GPIO 설정 초기화
    GPIO.cleanup()

# main()

