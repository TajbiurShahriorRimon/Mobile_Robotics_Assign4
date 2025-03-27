import rclpy
from rclpy.node import Node
from vslam_package.msg import ArucoMarker
from geometry_msgs.msg import PoseStamped

class VSLAM(Node):
    def __init__(self):
        super().__init__('vslam_node')
        self.subscription = self.create_subscription(ArucoMarker, 'aruco_detections', self.marker_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)

    def marker_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = msg.x
        pose_msg.pose.position.y = msg.y
        pose_msg.pose.orientation.w = 1.0  
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    vslam_node = VSLAM()
    rclpy.spin(vslam_node)
    vslam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

