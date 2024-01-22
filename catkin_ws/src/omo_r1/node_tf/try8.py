#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped

import math

class TFBroadcastNode:

    def __init__(self):
        rospy.init_node('tf_broadcast_node', anonymous=True)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize latest IMU and LiDAR scan messages
        self.latest_imu_msg = None
        self.latest_scan_msg = None

        # Initialize latest pose estimate
        self.latest_pose_msg = None

        # Subscribe to IMU, LiDAR scan, and 2D pose estimate topics
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/initialpose', PoseStamped, self.pose_estimate_callback)

    def imu_callback(self, msg):
        self.latest_imu_msg = msg

    def lidar_callback(self, msg):
        self.latest_scan_msg = msg

    def pose_estimate_callback(self, msg):
        self.latest_pose_msg = msg

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # Get latest IMU orientation quaternion and LiDAR center scan range and angle
            imu_orientation = self.latest_imu_msg.orientation if self.latest_imu_msg else None
            scan = self.latest_scan_msg
            if scan is not None:
                angle_increment = scan.angle_increment
                ranges = scan.ranges
                num_ranges = len(ranges)
                center_index = num_ranges // 2
                center_range = ranges[center_index]
                center_angle = (center_index * angle_increment) + scan.angle_min

                # Get latest pose estimate
                pose_estimate = self.latest_pose_msg.pose if self.latest_pose_msg else None
                if pose_estimate is not None:
                    # Use pose estimate as the initial position for the robot
                    robot_position = pose_estimate.position
                    robot_orientation = pose_estimate.orientation

                    # Calculate robot's x, y position based on LiDAR range and angle
                    robot_x = robot_position.x + center_range * math.cos(center_angle + math.pi/2.0)
                    robot_y = robot_position.y + center_range * math.sin(center_angle + math.pi/2.0)

                    # Create a new TransformStamped message from robot's position and orientation
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = current_time
                    tf_msg.header.frame_id = "map"
                    tf_msg.child_frame_id = "base_footprint"
                    tf_msg.transform.translation.x = robot_x
                    tf_msg.transform.translation.y = robot_y
                    tf_msg.transform.translation.z = 0.0
                    tf_msg.transform.rotation = robot_orientation if robot_orientation is not None else Quaternion()

                    # Publish the TF message
                    self.tf_broadcaster.sendTransform(tf_msg)

            rate.sleep()

if __name__ == '__main__':
    tf_broadcaster_node = TFBroadcastNode()
    tf_broadcaster_node.run()