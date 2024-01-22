#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(pose_msg):
    # Publish the 2D pose estimate as a transformation
    br = tf.TransformBroadcaster()
    br.sendTransform((pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, 0.0), 
                     tf.transformations.quaternion_from_euler(0, 0, pose_msg.pose.pose.orientation.z), 
                     rospy.Time.now(),
                     "odom",
                     "map")

    # Publish the pose message to cartographer_node
    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('cartographer_2d_pose_estimate')
    pub = rospy.Publisher('/cartographer_initial_pose', PoseWithCovarianceStamped, queue_size=1)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()
