#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.broadcaster import TransformBroadcaster

def pose_callback(msg):
    # Extract position and orientation from the message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

    # Broadcast the transform
    br = TransformBroadcaster()
    br.sendTransform((x, y, 0),
                     quaternion_from_euler(0, 0, yaw),
                     rospy.Time.now(),
                     "base_footprint",
                     "map")

if __name__ == '__main__':
    rospy.init_node('pose_to_tf')
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()
