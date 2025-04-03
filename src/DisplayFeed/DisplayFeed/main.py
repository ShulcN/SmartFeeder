import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.image_callback,
            10)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("ESP32 Camera", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()
    rclpy.spin(viewer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

