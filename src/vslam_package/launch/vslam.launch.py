from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RTAB-Map (VSLAM)
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{'frame_id': 'base_link'}]
        ),
        # ArUco Detector
        Node(
            package='vslam_package',
            executable='aruco_detector.py',
            name='aruco_detector'
        ),
        # VSLAM Node (your existing file)
        Node(
            package='vslam_package',
            executable='vslam_node.py',
            name='vslam_node'
        )
    ])
