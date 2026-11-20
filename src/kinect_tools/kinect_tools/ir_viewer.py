#!/usr/bin/env python3
"""IR viewer for Kinect v2 — displays infrared stream with normalized contrast."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IRViewer(Node):
    def __init__(self):
        super().__init__('ir_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/kinect/ir',
            self.listener_callback,
            1
        )
        self.get_logger().info('IR Viewer started. Waiting for IR images...')

    def listener_callback(self, msg):
        try:
            ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

            # Normalize uint16 to uint8 for display (stretch contrast)
            ir_min = ir_image.min()
            ir_max = ir_image.max()
            if ir_max > ir_min:
                normalized = ((ir_image.astype(np.float32) - ir_min) / (ir_max - ir_min) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(ir_image, dtype=np.uint8)

            cv2.imshow('Kinect v2 IR', normalized)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to process IR image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IRViewer()
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
