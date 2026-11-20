#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ByteMultiArray
from cv_bridge import CvBridge
import cv2

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        self.bridge = CvBridge()
        
        self.rgb_sub = self.create_subscription(Image, '/kinect/rgb', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/kinect/depth', self.depth_callback, 10)
        self.audio_sub = self.create_subscription(ByteMultiArray, 'kinect/audio/raw', self.audio_callback, 10)
        
        self.rgb_count = 0
        self.depth_count = 0
        self.audio_count = 0
        
        self.get_logger().info("Integration Test started. Monitoring all streams...")

    def rgb_callback(self, msg):
        self.rgb_count += 1
        if self.rgb_count % 30 == 0:
            self.get_logger().info(f"RGB frames: {self.rgb_count}")

    def depth_callback(self, msg):
        self.depth_count += 1
        if self.depth_count % 30 == 0:
            self.get_logger().info(f"Depth frames: {self.depth_count}")

    def audio_callback(self, msg):
        self.audio_count += 1
        if self.audio_count % 100 == 0:
            self.get_logger().info(f"Audio frames: {self.audio_count}")
            self.log_status()

    def log_status(self):
        self.get_logger().info(f"--- Stream Status ---")
        self.get_logger().info(f"RGB:   {self.rgb_count} frames")
        self.get_logger().info(f"Depth: {self.depth_count} frames")
        self.get_logger().info(f"Audio: {self.audio_count} frames")

def main(args=None):
    rclpy.init(args=args)
    node = IntegrationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
