import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.get_logger().info('Aruco Detector Node has been started.')

    def listener_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect ArUco markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for corner in corners:
                aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose
            camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.1, camera_matrix, dist_coeffs)

            for rvec, tvec in zip(rvecs, tvecs):
                aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Create and publish Pose message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'  # Change frame as needed
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[0][1]
                pose_msg.pose.position.z = tvec[0][2]

                # Publish Pose
                self.publisher.publish(pose_msg)

        cv2.imshow("Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

