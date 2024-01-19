#!/usr/bin/env python

import rospy
import actionlib
import RPi.GPIO as GPIO
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Int32

BUTTON = 20

cnt = 0
dcnt = 0

# Define the positions and orientations of the 4 destinations
destination1 = Pose(Point(2.5, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
destination2 = Pose(Point(2.5, 2.0, 0.0), Quaternion(0.0, 0.0, 0.7, 0.7))
destination3 = Pose(Point(0, 2.3, 0.0), Quaternion(0.0, 0.0, -1, 0))
destination4 = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, -0.7, 0.7))


# Define the position and orientation of the extra destination
extra_destination = Pose(Point(3.5, 4.3, 0.0), Quaternion(0.0, 0.0, 0.7, 0.7)) #the position of out 



# List of destinations
destinations = [destination1, destination2, destination3, destination4]

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def move_to_destination(destination):
    # Create an action client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Set up the move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = destination
    
    # Send the goal and wait for it to be achieved
    client.send_goal(goal)
    
    while(1):
      state = client.get_state()
      rospy.loginfo(state)
      if state == 3:
        break
    #result = client.wait_for_result(rospy.Duration.from_sec(30.0))

    # Check if the goal was achieved
    #if result:
    #    state = client.get_state()
     #   if state == actionlib.GoalStatus.SUCCEEDED:
      #      rospy.loginfo("Goal reached")
       #     #msg.data = 0
       #    #pub.publish(msg)
   #         #rospy.sleep(0.5)
    #    else:
     #       rospy.logwarn("Failed to reach goal, state: {}".format(state))
    #else:
      #  rospy.logwarn("Failed to reach goal within 30 seconds")
    
    #return result


def button_callback(channel):
    rospy.loginfo("Button pressed")
    
    #msg.data = 1
    #pub.publish(msg)
    #rospy.sleep(0.5)

    # Move the robot to the extra destination
    move_to_destination(extra_destination)
    move_to_destination(destination3)    
    # Move the robot back to the original 4 destinations
    #for destination in destinations:
        #move_to_destination(destination)

def main():

    try:
        # Move the robot to each destination in the list
        for destination in destinations:
            move_to_destination(destination3)

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node interrupted")

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('move_to_destinations_with_button')
    # Set up the button interrupt
    GPIO.add_event_detect(BUTTON, GPIO.RISING, callback=button_callback, bouncetime=300)

    msg = Int32()
    pub = rospy.Publisher('my_topic', Int32, queue_size=10)
    msg.data = 0
    pub.publish(msg)
    rospy.sleep(0.5)
    #main()