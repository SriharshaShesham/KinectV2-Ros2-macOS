#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import wave
import os
from datetime import datetime

class AudioSaver(Node):
    def __init__(self):
        super().__init__('audio_saver')
        
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('output_directory', '/tmp')
        
        self.sample_rate = self.get_parameter('sample_rate').as_int()
        self.channels = self.get_parameter('channels').as_int()
        self.output_dir = self.get_parameter('output_directory').as_string()
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate output filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = os.path.join(self.output_dir, f"kinect_audio_{timestamp}.wav")
        
        # Open WAV file for writing
        self.wav_file = wave.open(self.output_file, 'wb')
        self.wav_file.setnchannels(self.channels)
        self.wav_file.setsampwidth(2)  # 16-bit = 2 bytes
        self.wav_file.setframerate(self.sample_rate)
        
        self.subscription = self.create_subscription(
            ByteMultiArray,
            'kinect/audio/raw',
            self.listener_callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info(f"Audio Saver started. Saving to: {self.output_file}")

    def listener_callback(self, msg):
        try:
            self.frame_count += 1
            self.wav_file.writeframes(bytes(msg.data))
            
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"Audio frames saved: {self.frame_count}")
        except Exception as e:
            self.get_logger().error(f"Error saving audio: {e}")

    def __del__(self):
        if hasattr(self, 'wav_file'):
            self.wav_file.close()
            self.get_logger().info(f"Audio file saved: {self.output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = AudioSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
