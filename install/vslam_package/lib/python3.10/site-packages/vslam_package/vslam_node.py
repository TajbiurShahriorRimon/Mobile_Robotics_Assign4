import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VSLAMNode(Node):
    def __init__(self):
        super().__init__('vslam_node')
        self.publisher = self.create_publisher(String, '/vslam_status', 10)
        self.get_logger().info('VSLAM Node has been started.')

        # Start VSLAM process or any additional initialization here
        self.publish_status()

    def publish_status(self):
        msg = String()
        msg.data = "VSLAM process started!"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vslam_node = VSLAMNode()
    rclpy.spin(vslam_node)
    vslam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

