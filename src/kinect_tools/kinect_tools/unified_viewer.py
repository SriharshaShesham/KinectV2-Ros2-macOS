#!/usr/bin/env python3
"""
Unified Kinect viewer: 2x2 grid (RGB, Depth, IR, Blank)
Combines rgb_viewer, depth_viewer, ir_viewer logic.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class UnifiedViewer(Node):
    def __init__(self):
        super().__init__('unified_viewer')
        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.ir_img = None
        # Subscriptions
        self.create_subscription(CompressedImage, '/kinect/rgb/compressed', self.rgb_callback, 1)
        self.create_subscription(Image, '/kinect/depth', self.depth_callback, 1)
        self.create_subscription(Image, '/kinect/ir', self.ir_callback, 1)
        self.get_logger().info('Unified Viewer started. Waiting for images...')

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.rgb_img = cv2.resize(cv_image, (480, 480)) if cv_image is not None else None
        except Exception as e:
            self.get_logger().error(f'Failed to decode compressed image: {e}')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            valid = depth_image[depth_image > 0]
            if valid.size > 0:
                depth_min = valid.min()
                depth_max = valid.max()
            else:
                depth_min, depth_max = 0, 1
            if depth_max > depth_min:
                normalized = ((depth_image.astype(np.float32) - depth_min) / (depth_max - depth_min) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(depth_image, dtype=np.uint8)
            colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            self.depth_img = cv2.resize(colored, (480, 480))
        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {e}')

    def ir_callback(self, msg):
        try:
            ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            ir_min = ir_image.min()
            ir_max = ir_image.max()
            if ir_max > ir_min:
                normalized = ((ir_image.astype(np.float32) - ir_min) / (ir_max - ir_min) * 255).astype(np.uint8)
            else:
                normalized = np.zeros_like(ir_image, dtype=np.uint8)
            ir_bgr = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)
            self.ir_img = cv2.resize(ir_bgr, (480, 480))
        except Exception as e:
            self.get_logger().error(f'Failed to process IR image: {e}')

    def show(self):
        QUAD_W, QUAD_H = 480, 480
        composite = np.zeros((QUAD_H * 2, QUAD_W * 2, 3), dtype=np.uint8)
        BLACK_QUAD = np.zeros((QUAD_H, QUAD_W, 3), dtype=np.uint8)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            composite[:] = 40  # dark grey
            # Top-left — RGB
            composite[0:QUAD_H, 0:QUAD_W] = self.rgb_img if self.rgb_img is not None else BLACK_QUAD
            cv2.putText(composite, 'RGB', (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            # Top-right — Depth
            composite[0:QUAD_H, QUAD_W:QUAD_W * 2] = self.depth_img if self.depth_img is not None else BLACK_QUAD
            cv2.putText(composite, 'Depth', (QUAD_W + 10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            # Bottom-left — IR
            composite[QUAD_H:QUAD_H * 2, 0:QUAD_W] = self.ir_img if self.ir_img is not None else BLACK_QUAD
            cv2.putText(composite, 'IR', (10, QUAD_H + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            # Bottom-right — Blank
            cv2.putText(composite, 'Blank', (QUAD_W + 10, QUAD_H + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
            cv2.imshow('Unified Kinect Viewer (2x2: RGB | Depth | IR | Blank)', composite)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    viewer = UnifiedViewer()
    viewer.show()
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

