#!/usr/bin/env python3
"""Diagnose what's actually being published on each topic"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ByteMultiArray
from cv_bridge import CvBridge
import time

class TopicDiagnostic(Node):
    def __init__(self):
        super().__init__('topic_diagnostic')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.rgb_sub = self.create_subscription(Image, '/kinect/rgb', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(Image, '/kinect/depth', self.depth_callback, 1)
        self.ir_sub = self.create_subscription(Image, '/kinect/ir', self.ir_callback, 1)
        self.audio_sub = self.create_subscription(ByteMultiArray, '/kinect/audio/raw', self.audio_callback, 1)
        
        self.counts = {'rgb': 0, 'depth': 0, 'ir': 0, 'audio': 0}
        self.last_time = time.time()
        self.get_logger().info("Diagnostic node started. Monitoring topics...")
        
    def rgb_callback(self, msg):
        self.counts['rgb'] += 1
        if self.counts['rgb'] % 30 == 0:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(
                f"RGB: shape={rgb_img.shape}, dtype={rgb_img.dtype}, "
                f"encoding={msg.encoding}, step={msg.step}")
    
    def depth_callback(self, msg):
        self.counts['depth'] += 1
        if self.counts['depth'] % 30 == 0:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.get_logger().info(
                f"Depth: shape={depth_img.shape}, dtype={depth_img.dtype}, "
                f"encoding={msg.encoding}, min={depth_img.min():.2f}, max={depth_img.max():.2f}")
    
    def ir_callback(self, msg):
        self.counts['ir'] += 1
        if self.counts['ir'] % 30 == 0:
            ir_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            self.get_logger().info(
                f"IR: shape={ir_img.shape}, dtype={ir_img.dtype}, "
                f"encoding={msg.encoding}")
    
    def audio_callback(self, msg):
        self.counts['audio'] += 1
        if self.counts['audio'] == 1:
            # Log first audio packet in detail
            self.get_logger().info(
                f"Audio (first packet): data_len={len(msg.data)}, "
                f"data[0:8]={msg.data[0:8]}, layout_dim_size={msg.layout.dim[0].size if msg.layout.dim else 'none'}")
        
        # Show statistics every 30 packets
        if self.counts['audio'] % 30 == 0:
            # Try to understand the data format
            import numpy as np
            try:
                audio_bytes = bytes(bytearray(msg.data))
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
                self.get_logger().info(
                    f"Audio: {len(msg.data)} bytes = {len(audio_int16)} int16 samples, "
                    f"min={audio_int16.min()}, max={audio_int16.max()}, "
                    f"mean={np.mean(audio_int16):.1f}, std={np.std(audio_int16):.1f}")
            except Exception as e:
                self.get_logger().error(f"Audio analysis error: {e}")
        
        # Print FPS every 3 seconds
        now = time.time()
        if now - self.last_time > 3:
            elapsed = now - self.last_time
            self.get_logger().warning(
                f"FRAME RATES (per {elapsed:.1f}s): "
                f"RGB={self.counts['rgb']/(elapsed):.1f}fps, "
                f"Depth={self.counts['depth']/(elapsed):.1f}fps, "
                f"IR={self.counts['ir']/(elapsed):.1f}fps, "
                f"Audio={self.counts['audio']/(elapsed):.1f}fps")
            self.counts = {'rgb': 0, 'depth': 0, 'ir': 0, 'audio': 0}
            self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = TopicDiagnostic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
