#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
import numpy as np
from scipy.linalg import block_diag
import tf.transformations as tr
import math
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class TFBroadcastNode:

    def __init__(self):
        rospy.init_node('tf_broadcast_node', anonymous=True)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize latest LiDAR scan and costmap messages
        self.latest_scan_msg = None
        self.latest_costmap_msg = None
        self.latest_amcl_msg = None

        # Subscribe to LiDAR scan and costmap topics
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)  # Subscribe to 2D Pose Estimate topic
       
        self.filter = KalmanFilter(dim_x=3, dim_z=1)
        self.filter.x = np.array([0.0, 0.0, 0.0])  # initial state (x, y, yaw)
        self.filter.F = np.array([[1.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0],
                                  [0.0, 0.0, 1.0]])  # state transition matrix
        self.filter.H = np.array([[0.0, 0.0, 1.0]])  # measurement function
        self.filter.P *= 10  # covariance matrix
        self.filter.R = np.array([[0.1]])  # measurement noise covariance matrix
        self.filter.Q = Q_discrete_white_noise(dim=3, dt=1.0, var=0.1)   
                
        
    def lidar_callback(self, msg):
        self.latest_scan_msg = msg
    
    # Apply Kalman filter to robot's pose

    
    def costmap_callback(self, msg):
        self.latest_costmap_msg = msg


    def amcl_callback(self, msg):
        self.latest_amcl_msg = msg

    def run(self):
        rate = rospy.Rate(10.0)
        n = 0
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

        # Get latest 2D Pose Estimate message
            amcl = self.latest_amcl_msg
            
            if amcl is not None:
              # Get robot's x, y position and yaw angle from 2D Pose Estimate message
                if n == 0:
                    robot_x = amcl.pose.pose.position.x
                    robot_y = amcl.pose.pose.position.y
                    robot_yaw = tr.euler_from_quaternion([amcl.pose.pose.orientation.x, 
                                                          amcl.pose.pose.orientation.y, 
                                                          amcl.pose.pose.orientation.z, 
                                                          amcl.pose.pose.orientation.w])[2]
                    
                    n=0
                else:
                    robot_x=robot_x
                    robot_y=robot_y
                    robot_yaw=robot_yaw

                robot_quat = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, robot_yaw))
    
            # Get latest LiDAR center scan range and angle
                scan = self.latest_scan_msg
                if scan is not None:
                    angle_increment = scan.angle_increment
                    ranges = scan.ranges
                    num_ranges = len(ranges)
                    center_index = num_ranges // 2
                    center_range = ranges[center_index]
                    center_angle = (center_index * angle_increment) + scan.angle_min
                    

                # Get latest costmap message
                    costmap = self.latest_costmap_msg
                    
                    
                    if costmap is not None:
                    # Calculate robot's x, y position based on LiDAR range and angle using
                    # trigonometry
                        z = np.array([[center_range]])

                        self.filter.predict()
                        self.filter.update(z)
                        robot_x, robot_y, robot_yaw = self.filter.x
                        rospy.loginfo("{}".format(robot_x))
                        
                    # Calculate robot's z position based on costmap
                        costmap_info = costmap.info
                        x_index = int((robot_x - costmap_info.origin.position.x) / costmap_info.resolution)
                        y_index = int((robot_y - costmap_info.origin.position.y) / costmap_info.resolution)
                        costmap_index = x_index + (y_index * costmap_info.width)
                        costmap_value = costmap.data[costmap_index]
                        robot_z = costmap_value * costmap_info.resolution
                        


                    # Broadcast TF transform from map to robot
                        t = TransformStamped()
                        
                        t.header.stamp = current_time
                        t.header.frame_id = "map"
                        t.child_frame_id = "base_footprint"
                        t.transform.translation.x = robot_x
                        t.transform.translation.y = robot_y
                        t.transform.translation.z = robot_z
                        t.transform.rotation = robot_quat
                        self.tf_broadcaster.sendTransform(t)

            rate.sleep()

if __name__ == '__main__':
    tf_broadcaster_node = TFBroadcastNode()
    tf_broadcaster_node.run()