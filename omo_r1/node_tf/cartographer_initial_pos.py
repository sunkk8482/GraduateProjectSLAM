#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(pose_msg):
    # Publish the 2D pose estimate as a transformation
    br = tf.TransformBroadcaster()
    br.sendTransform((1.0, 0.0, 0.0), 
                     (0.0, 0.0, 0.0, 1.0), 
                     rospy.Time.now(),
                     "base_footprint",
                     "map")

    # Publish the pose message to cartographer_node
    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('cartographer_2d_pose_estimate')
    pub = rospy.Publisher('/pose_estimate', PoseWithCovarianceStamped, queue_size=1)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_callback)
    rospy.spin()



