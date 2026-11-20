#!/usr/bin/env python3
"""
kinect_tools — Point Cloud Node

Subscribes to Kinect v2 depth (mono16) + depth camera intrinsics.
Optionally colorizes from compressed RGB.
Publishes sensor_msgs/PointCloud2 on kinect/pointcloud.

Architecture:
  kinect_node (C++) → depth + camera_info → THIS NODE → PointCloud2 → viewer / navigation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')
        # Exit when orphaned: killing the kinect launch left this process
        # running under parent 1 (five stale copies found, 2026-09-30).
        import os as _os
        self._parent = _os.getppid()
        self.create_timer(2.0, lambda: _os.getppid() != self._parent and _os._exit(0))

        # Parameters
        self.declare_parameter('downsample', 4)        # skip every Nth pixel (4 → ~13K points)
        self.declare_parameter('max_depth_m', 4.5)     # Kinect v2 max reliable range
        self.declare_parameter('min_depth_m', 0.5)     # Kinect v2 min range
        self.declare_parameter('use_color', True)      # colorize points from RGB
        self.declare_parameter('target_hz', 5.0)       # point cloud publish rate

        self.downsample = self.get_parameter('downsample').value
        self.max_depth = self.get_parameter('max_depth_m').value
        self.min_depth = self.get_parameter('min_depth_m').value
        self.use_color = self.get_parameter('use_color').value
        target_hz = self.get_parameter('target_hz').value

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_rgb = None  # Registered RGB (512x424, aligned to depth)
        self.intrinsics_logged = False

        # Pre-computed pixel grids (set on first camera_info)
        self.u_grid = None
        self.v_grid = None
        self.grid_shape = None

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, 'kinect/pointcloud', 1)

        # Subscribers
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info',
            self.camera_info_callback, 1)

        self.depth_sub = self.create_subscription(
            Image, 'kinect/depth',
            self.depth_callback, 1)

        if self.use_color:
            self.rgb_sub = self.create_subscription(
                Image, 'kinect/rgb/registered',
                self.rgb_callback, 1)

        # Rate limiter
        self._publish_interval = 1.0 / target_hz
        self._last_publish = 0.0

        self.get_logger().info(
            f'PointCloud node started (downsample={self.downsample}, '
            f'range={self.min_depth}-{self.max_depth}m, '
            f'color={self.use_color}, rate={target_hz}Hz)')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        if not self.intrinsics_logged:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.get_logger().info(
                f'Depth intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} '
                f'({msg.width}x{msg.height})')
            self.intrinsics_logged = True

    def rgb_callback(self, msg):
        # Registered RGB is already 512x424 aligned to depth frame
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _build_pixel_grids(self, h, w):
        """Pre-compute downsampled pixel coordinate grids (done once)."""
        ds = self.downsample
        v_coords = np.arange(0, h, ds)
        u_coords = np.arange(0, w, ds)
        self.u_grid, self.v_grid = np.meshgrid(u_coords, v_coords)
        self.grid_shape = (h, w)
        self.get_logger().info(
            f'Pixel grids built: {h}x{w} → {self.u_grid.shape[0]}x{self.u_grid.shape[1]} '
            f'({self.u_grid.size} points/frame)')

    def depth_callback(self, msg):
        # Lazy publishing — skip if nobody is listening
        if self.pc_pub.get_subscription_count() == 0:
            return

        # Wait for intrinsics
        if self.camera_info is None:
            return

        # Rate limit
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_publish < self._publish_interval:
            return
        self._last_publish = now

        # Decode depth (mono16, uint16, millimeters)
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        h, w = depth.shape

        # Build pixel grids on first frame (or if resolution changes)
        if self.grid_shape != (h, w):
            self._build_pixel_grids(h, w)

        # Get intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Sample depth at downsampled positions → convert mm to meters
        depth_sampled = depth[self.v_grid, self.u_grid].astype(np.float32) / 1000.0

        # Filter valid depths
        valid = (depth_sampled > self.min_depth) & (depth_sampled < self.max_depth)

        z = depth_sampled[valid]
        u = self.u_grid[valid].astype(np.float32)
        v = self.v_grid[valid].astype(np.float32)

        if z.size == 0:
            return

        # Back-project to 3D (pinhole camera model)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Build PointCloud2 message
        has_color = self.use_color and self.latest_rgb is not None

        if has_color:
            # Registered RGB is already aligned to depth (512x424) — direct lookup
            rgb = self.latest_rgb
            b = rgb[self.v_grid[valid], self.u_grid[valid], 0].astype(np.uint32)
            g = rgb[self.v_grid[valid], self.u_grid[valid], 1].astype(np.uint32)
            r = rgb[self.v_grid[valid], self.u_grid[valid], 2].astype(np.uint32)

            # Filter out black pixels (parallax/occlusion — depth sees it but RGB can't)
            bright = (r + g + b) > 30
            x, y, z = x[bright], y[bright], z[bright]
            r, g, b = r[bright], g[bright], b[bright]

            if z.size == 0:
                return

            # Pack RGB into float32 (ROS PointCloud2 convention)
            rgb_packed = ((r << 16) | (g << 8) | b).astype(np.uint32)
            rgb_float = rgb_packed.view(np.float32)

            # XYZRGB: 16 bytes per point
            points = np.column_stack([x, y, z, rgb_float]).astype(np.float32)
            point_step = 16
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
        else:
            # XYZ only: 12 bytes per point
            points = np.column_stack([x, y, z]).astype(np.float32)
            point_step = 12
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        # Construct PointCloud2 directly from numpy (fast path, no create_cloud)
        pc_msg = PointCloud2()
        pc_msg.header = msg.header
        pc_msg.header.frame_id = 'kinect_depth_optical_frame'
        pc_msg.height = 1
        pc_msg.width = len(x)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = point_step
        pc_msg.row_step = point_step * pc_msg.width
        pc_msg.is_dense = True
        pc_msg.data = points.tobytes()

        self.pc_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
