#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')
        self.bridge = CvBridge()
        # QoS depth=1: always process the latest frame, drop stale ones
        self.subscription = self.create_subscription(
            Image,
            '/kinect/depth',
            self.listener_callback,
            1
        )
        self.get_logger().info("Depth Viewer started. Waiting for depth images...")

    def listener_callback(self, msg):
        try:
            # Depth now arrives as mono16 (uint16 mm) — much smaller than 32FC1
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            
            # Normalize uint16 range for visualization
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
            
            # Apply colormap (heatmap)
            colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            
            cv2.imshow("Kinect v2 Depth Stream", colored)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthViewer()
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
