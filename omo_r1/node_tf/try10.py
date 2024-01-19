#!/usr/bin/env python

import rospy
import actionlib
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

BUTTON = 20

# Define the positions and orientations of the 4 destinations
destination1 = Pose(Point(0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
destination2 = Pose(Point(0.0, 0.5, 0.0), Quaternion(0.0, 0.0, 0.707, 0.707))
destination3 = Pose(Point(-0.5, 0.0, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0))
destination4 = Pose(Point(0.0, -0.5, 0.0), Quaternion(0.0, 0.0, -0.707, 0.707))

# Define the position and orientation of the extra destination
extra_destination = Pose(Point(-1.68394994736, -1.23710918427, 0.0), Quaternion(0.0, 0.0, 0.985356028573, 0.170509521599))

# List of destinations
destinations = [destination1, destination2, destination3, destination4]

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def move_to_destination(destination):
    # Create an action client for the move_base action server
    rospy.loginfo("aa")
    
    

def button_callback(channel):
    rospy.loginfo("Button pressed")
    
    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    # Cancel the current goal to stop the robot from moving
    client.cancel_goal()
    rospy.loginfo("Current goal canceled")
    
 
    
    goal = MoveBaseGoal()
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reset")
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = extra_destination
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("New goal set")

def main():
    try:
        # Initialize the ROS node
        rospy.init_node('move_to_destinations_with_button')

        # Set up the button interrupt
        
        

        # Move the robot to each destination in the list
        for destination in destinations:
            move_to_destination(destination)

        rospy.spin();
    
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")

    finally:
        GPIO.cleanup()


GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_callback, bouncetime=300)
if __name__ == '__main__':
    main()
