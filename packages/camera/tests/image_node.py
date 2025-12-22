import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (255, 0, 0) 
        cv2.putText(img, 'ROS2 Image Test', (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        msg = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpg')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing compressed image')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
