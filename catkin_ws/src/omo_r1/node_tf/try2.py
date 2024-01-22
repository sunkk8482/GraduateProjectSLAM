#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped


class TFBroadcastNode:

    def run(self):
    
        while not rospy.is_shutdown():
            rospy.init_node('my_node')

            msg = TransformStamped()
  
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.child_frame_id = "base_footprint"
            msg.transform.translation.x = 0.8070316314697266
            msg.transform.translation.y = -0.7719531059265137

            msg.transform.translation.z = 0.0
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z =  0.9946968411987187
            msg.transform.rotation.w = 0.10285034812430631
    
            pub = rospy.Publisher('/tf', TransformStamped, queue_size=10)
            pub.publish(msg)

if __name__ == '__main__':
    tf_broadcaster_node = TFBroadcastNode()
    tf_broadcaster_node.run()

