#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist

class ImuToOdom:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        self.last_time = rospy.Time.now()
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

    def imu_callback(self, msg):
        # Get time delta
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Get orientation and velocity from IMU
        q = msg.orientation
        v = msg.angular_velocity

        # Predict pose and velocity
        q_predict = tf_conversions.transformations.quaternion_from_euler(v.x * dt, v.y * dt, v.z * dt)
        q_new = tf_conversions.transformations.quaternion_multiply(q, q_predict)
        t_predict = Twist()
        t_predict.angular = v
        t_predict.linear.x = 0
        t_predict.linear.y = 0
        t_predict.linear.z = 0

        # Create a new TransformStamped message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(*q_new)

        # Publish the TF message
        self.tf_broadcaster.sendTransform(tf_msg)

        # Update odometry message
        self.odom.header.stamp = current_time
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation = Quaternion(*q_new)
        self.odom.twist.twist = t_predict

        # Publish the odometry message
        self.odom_pub.publish(self.odom)

        # Update time stamp
        self.last_time = current_time


if __name__ == '__main__':
    rospy.init_node('imu_to_odom', anonymous=True)
    imu_to_odom = ImuToOdom()
    rospy.spin()