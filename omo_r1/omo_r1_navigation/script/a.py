#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_node')

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create a static transform from base_footprint to map
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"
    static_transform_stamped.child_frame_id = "base_footprint"

    # Set the translation and rotation of the transform
    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0

    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0

    # Broadcast the transform
    tf_broadcaster.sendTransform(static_transform_stamped)

    rospy.spin()