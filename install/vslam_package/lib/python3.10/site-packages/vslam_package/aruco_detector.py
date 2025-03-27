import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from vslam_package.msg import ArucoMarker  # Ensure this custom message exists

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(ArucoMarker, 'aruco_detections', 10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            for i in range(len(ids)):
                marker_msg = ArucoMarker()
                marker_msg.id = int(ids[i])
                marker_msg.x = float(corners[i][0][0][0])
                marker_msg.y = float(corners[i][0][0][1])
                self.publisher_.publish(marker_msg)

        aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

