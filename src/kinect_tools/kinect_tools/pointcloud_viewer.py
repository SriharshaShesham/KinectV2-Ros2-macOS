#!/usr/bin/env python3
"""
Point Cloud Viewer — Bird's-eye (top-down) + front view using OpenCV.

Subscribes to kinect/pointcloud (PointCloud2).
No extra dependencies — uses the same OpenCV pattern as RGB/Depth/IR viewers.

Bird's-eye view: X (left-right) vs Z (depth) — useful for obstacle mapping.
Front view: X (left-right) vs Y (up-down) — useful for scene understanding.
Points colored by depth (Z) or by RGB if available.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import cv2
import numpy as np


class PointCloudViewer(Node):
    def __init__(self):
        super().__init__('pointcloud_viewer')

        # View parameters
        self.declare_parameter('view_range_m', 5.0)    # max depth range in meters
        self.declare_parameter('side_range_m', 3.0)     # horizontal range (±m)
        self.declare_parameter('height_range_m', 2.0)   # vertical range (±m)
        self.declare_parameter('resolution', 500)        # canvas size in pixels

        self.view_range = self.get_parameter('view_range_m').value
        self.side_range = self.get_parameter('side_range_m').value
        self.height_range = self.get_parameter('height_range_m').value
        self.res = self.get_parameter('resolution').value

        self.subscription = self.create_subscription(
            PointCloud2, 'kinect/pointcloud',
            self.pc_callback, 1)

        self.frame_count = 0
        self.get_logger().info(
            f'PointCloud Viewer started (range={self.view_range}m, '
            f'side=±{self.side_range}m, height=±{self.height_range}m)')

    def pc_callback(self, msg):
        n_points = msg.width * msg.height
        if n_points == 0:
            return

        point_step = msg.point_step
        has_rgb = point_step >= 16

        # Fast numpy parse: interpret raw buffer as float32 array
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, point_step // 4)
        x = arr[:, 0]
        y = arr[:, 1]
        z = arr[:, 2]

        # Filter NaN/Inf
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]

        if len(x) == 0:
            return

        # Extract RGB if available, filter out dark pixels
        if has_rgb:
            rgb_float = arr[valid, 3]
            rgb_int = rgb_float.view(np.uint32)
            r = ((rgb_int >> 16) & 0xFF).astype(np.uint8)
            g = ((rgb_int >> 8) & 0xFF).astype(np.uint8)
            b = (rgb_int & 0xFF).astype(np.uint8)
            # Filter out near-black pixels (from depth/color misalignment)
            brightness = r.astype(int) + g.astype(int) + b.astype(int)
            bright_enough = brightness > 30  # skip very dark pixels
            x, y, z = x[bright_enough], y[bright_enough], z[bright_enough]
            r, g, b = r[bright_enough], g[bright_enough], b[bright_enough]
            point_colors = np.column_stack([b, g, r])  # BGR for OpenCV
        else:
            point_colors = None

        if len(x) == 0:
            return

        # Sort far-to-near (painter's algorithm: near points drawn last = on top)
        order = np.argsort(-z)
        x, y, z = x[order], y[order], z[order]
        if point_colors is not None:
            point_colors = point_colors[order]

        res = self.res
        pad = 10  # gap between views
        canvas_w = res * 2 + pad
        canvas_h = res
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        # ===== LEFT PANEL: Bird's-eye view (X vs Z) =====
        bev = canvas[:, :res]
        self._draw_birdseye(bev, x, y, z, point_colors, res)

        # ===== RIGHT PANEL: Front view (X vs Y) =====
        front = canvas[:, res + pad:]
        self._draw_frontview(front, x, y, z, point_colors, res)

        # Separator line
        canvas[:, res:res + pad] = (40, 40, 40)

        # Global info
        self.frame_count += 1
        cv2.putText(canvas, f'Frame: {self.frame_count}  Points: {len(x)}',
                    (10, canvas_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow('Point Cloud Viewer', canvas)
        cv2.waitKey(1)

    def _draw_birdseye(self, panel, x, y, z, colors, res):
        """Bird's-eye / Top-down view: X (horizontal) vs Z (depth into scene)."""
        # Map X: [-side_range, side_range] → [0, res]
        # Map Z: [0, view_range] → [res, 0]  (near at bottom)
        px = ((x / (2 * self.side_range) + 0.5) * res).astype(int)
        pz = ((1.0 - z / self.view_range) * res).astype(int)

        mask = (px >= 0) & (px < res) & (pz >= 0) & (pz < res)
        px, pz = px[mask], pz[mask]

        if colors is not None:
            c = colors[mask]
        else:
            # Color by depth (Z)
            z_masked = z[mask]
            z_norm = np.clip(z_masked / self.view_range, 0, 1)
            c = cv2.applyColorMap(
                (z_norm * 255).astype(np.uint8).reshape(-1, 1),
                cv2.COLORMAP_TURBO).reshape(-1, 3)

        # Draw with 2x2 pixel size for better visibility
        for dy in range(2):
            for dx in range(2):
                pz_s = np.clip(pz + dy, 0, res - 1)
                px_s = np.clip(px + dx, 0, res - 1)
                panel[pz_s, px_s] = c

        # Grid lines (depth)
        for d in range(1, int(self.view_range) + 1):
            row = int((1.0 - d / self.view_range) * res)
            if 0 <= row < res:
                cv2.line(panel, (0, row), (res, row), (30, 30, 30), 1)
                cv2.putText(panel, f'{d}m', (5, row - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 80, 80), 1)

        # Center line
        cv2.line(panel, (res // 2, 0), (res // 2, res), (30, 30, 30), 1)

        # Title
        cv2.putText(panel, "Bird's Eye (X-Z)", (res // 2 - 60, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

    def _draw_frontview(self, panel, x, y, z, colors, res):
        """Front view: X (horizontal) vs Y (vertical)."""
        # Map X: [-side_range, side_range] → [0, res]
        # Map Y: [-height_range, height_range] → [0, res]  (Y up in camera = down in image)
        px = ((x / (2 * self.side_range) + 0.5) * res).astype(int)
        py = ((y / (2 * self.height_range) + 0.5) * res).astype(int)

        mask = (px >= 0) & (px < res) & (py >= 0) & (py < res)
        px, py = px[mask], py[mask]

        if colors is not None:
            c = colors[mask]
        else:
            z_masked = z[mask]
            z_norm = np.clip(z_masked / self.view_range, 0, 1)
            c = cv2.applyColorMap(
                (z_norm * 255).astype(np.uint8).reshape(-1, 1),
                cv2.COLORMAP_TURBO).reshape(-1, 3)

        # Draw with 2x2 pixel size for better visibility
        for dy in range(2):
            for dx in range(2):
                py_s = np.clip(py + dy, 0, res - 1)
                px_s = np.clip(px + dx, 0, res - 1)
                panel[py_s, px_s] = c

        # Grid
        cv2.line(panel, (res // 2, 0), (res // 2, res), (30, 30, 30), 1)
        cv2.line(panel, (0, res // 2), (res, res // 2), (30, 30, 30), 1)

        # Title
        cv2.putText(panel, 'Front View (X-Y)', (res // 2 - 60, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudViewer()
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
