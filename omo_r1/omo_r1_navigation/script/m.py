#!/usr/bin/env python


import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Get pose information from Odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()

    # Publish transform from base_link to odom
    br.sendTransform(
        (position.x, position.y, position.z),                    # Translation (x, y, z)
        (orientation.x, orientation.y, orientation.z, orientation.w), # Rotation (quaternion)
        rospy.Time.now(),                                        # Timestamp
        "base_link",                                             # Child frame ID
        "odom"                                                   # Parent frame ID
    )

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('robot_tf_broadcaster')

    # Subscribe to Odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Start the node
    rospy.spin()