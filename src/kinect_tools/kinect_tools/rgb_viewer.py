#!/usr/bin/env python3
"""RGB viewer subscribing to JPEG-compressed topic for low bandwidth."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class RGBViewer(Node):
    def __init__(self):
        super().__init__('rgb_viewer')
        # Subscribe to JPEG compressed: ~50-100KB instead of ~1.5MB per frame
        self.subscription = self.create_subscription(
            CompressedImage,
            '/kinect/rgb/compressed',
            self.listener_callback,
            1
        )
        self.get_logger().info('RGB Viewer started (compressed). Waiting for images...')

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                cv2.imshow('Kinect v2 RGB', cv_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to decode compressed image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RGBViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
