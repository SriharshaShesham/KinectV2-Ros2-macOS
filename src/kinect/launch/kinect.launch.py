import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Optional overrides (ROS params-file format, one section per node name).
# Listed AFTER each node's defaults, so a value there overrides the default.
# Path can be changed with the KINECT_PARAMS environment variable.
PARAMS_FILE = os.path.expanduser(os.environ.get('KINECT_PARAMS', '~/.kinectv2/params.yaml'))
OVERRIDES = [PARAMS_FILE] if os.path.exists(PARAMS_FILE) else []

# PUBLISHERS only. Viewers / consoles / your own nodes are separate processes
# that subscribe to these topics — start as many of them as you like.
#   ros2 launch kinect kinect.launch.py                    # driver + audio + point cloud
#   ros2 launch kinect kinect.launch.py pointcloud:=false audio:=false


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('audio', default_value='true',
                              description='Publish the mic array (kinect/audio/*)'),
        DeclareLaunchArgument('pointcloud', default_value='true',
                              description='Publish kinect/pointcloud (subscribes to depth)'),
        # Kinect RGB + Depth + IR node (C++, libfreenect2)
        Node(
            package='kinect',
            executable='kinect_node',
            name='kinect_node',
            output='screen',
            parameters=[{
                'pipeline': 'cl',      # cl | cpu
                'publish_ir': True,
                'rgb_scale': 0.5,      # 0.5 = 960x540 (balanced), 0.33 = 640x360 (low latency), 1.0 = full
                'serial': '',          # set if multiple devices
                'target_fps': 30.0,    # was 15 (bandwidth cap from the multi-viewer era);
                                       # person tracking wants frames — every correction
                                       # waits for one, and YOLOX has 25fps capacity (2026-09-28)
                'jpeg_quality': 80,    # JPEG quality for compressed RGB (1-100)
            }] + OVERRIDES
        ),
        # Audio capture from Kinect mic array (Python, sounddevice via USB)
        Node(
            package='kinect_tools',
            executable='audio_publisher',
            name='audio_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('audio')),
            # exits on an audio stall (USB reset at driver start); come back
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'sample_rate': 16000,
                'channels': 4,
                'chunk_ms': 50,
                'device_name': 'Xbox NUI Sensor',
                'mic_spacing_m': 0.226,   # outer pair; measured 2026-10-02 (was 0.113 = half)
            }] + OVERRIDES
        ),
        # Point cloud generation (Python, kinect_tools)
        Node(
            package='kinect_tools',
            executable='pointcloud_node',
            name='pointcloud_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('pointcloud')),
            parameters=[{
                'downsample': 4,        # every 4th pixel → ~13K points
                'max_depth_m': 4.5,     # Kinect v2 max reliable range
                'min_depth_m': 0.5,     # Kinect v2 min range
                'use_color': True,      # colorize from RGB
                'target_hz': 5.0,       # 5Hz point cloud rate
            }] + OVERRIDES
        ),
    ])
