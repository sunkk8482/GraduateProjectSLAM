#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def imu_publisher():
    rospy.init_node('omo_r1_imu_node', anonymous=True)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rate = rospy.Rate(50)  # 50Hz

    while not rospy.is_shutdown():
        # TODO: Read data from MPU6050 and publish Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()

        # Set orientation to identity quaternion (no rotation)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        # Set angular velocity to zero (since robot only moves in 2D)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        imu_pub.publish(imu_msg)s
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
