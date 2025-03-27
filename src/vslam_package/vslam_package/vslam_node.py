#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vslam_package.msg import ArucoMarker
import numpy as np

class VSLAMNode(Node):
    def __init__(self):
        super().__init__('vslam_node')
        
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.sub_aruco = self.create_subscription(ArucoMarker, '/aruco_markers', self.aruco_callback, 10)
        
        # Storage for markers (ID: corners)
        self.markers = {}  
        
    def odom_callback(self, msg):
        """Callback for VSLAM odometry."""
        # TODO: Fuse with ArUco data to improve localization
        pass

    def aruco_callback(self, msg):
        """Callback for ArUco marker detection."""
        if msg.id in [1, 2, 3, 4]:  # Only track IDs 1-4
            self.markers[msg.id] = np.array(msg.corners).reshape(4, 2)  # Store corners as 4x2 array
            self.get_logger().info(f"Updated marker {msg.id} pose")

            # If all 4 markers detected, compute center
            if len(self.markers) == 4:
                self.compute_environment_center()

    def compute_environment_center(self):
        """Calculate center point from 4 ArUco markers."""
        centers = []
        for corners in self.markers.values():
            centers.append(np.mean(corners, axis=0))  # Center of each marker
        
        env_center = np.mean(centers, axis=0)  # Global center
        self.get_logger().info(f"Environment center: {env_center}")
        # TODO: Publish this to a topic for navigation

def main():
    rclpy.init()
    node = VSLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
