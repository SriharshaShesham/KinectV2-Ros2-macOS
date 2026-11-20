"""ROS side of the console: a pure SUBSCRIBER of the kinect topics.

Run as many consoles (or viewers, or your own nodes) as you like — each is
just another subscriber. Every console node gets a unique name.

Subscriptions are lazy and reference-counted: a stream is subscribed only
while at least one browser is watching it. That matters because the kinect
driver only encodes/registers a stream while it has subscribers.
"""

import os
import threading
import time
from typing import Callable, Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import ByteMultiArray, Float32

from . import convert

# RELIABLE for big raw images: a ~1 MB frame over BEST_EFFORT loses one UDP
# fragment and the whole frame is dropped (learned the hard way).
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
AUDIO_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# source id -> (topic, message type, converter, keeps depth for readout)
SOURCES = {
    "color": ("/kinect/rgb/compressed", CompressedImage, convert.passthrough_jpeg, False),
    "ir": ("/kinect/ir", Image, convert.ir_to_jpeg, False),
    "depth": ("/kinect/depth", Image, convert.depth_to_jpeg, True),
    "registered": ("/kinect/rgb/registered", Image, convert.bgr_to_jpeg, False),
    "aligned": ("/kinect/depth/color_aligned", Image, convert.depth_to_jpeg, True),
}


class ConsoleBridge(Node):
    def __init__(self, on_frame: Callable[[str, bytes], None],
                 on_audio: Callable[[dict], None]):
        super().__init__("kinect_console_%d" % os.getpid())
        self._on_frame = on_frame
        self._on_audio = on_audio
        self._lock = threading.Lock()
        self._video: Dict[str, dict] = {}
        self._depth: Dict[str, tuple] = {}     # src -> (uint16 array, monotonic time)
        self._audio_subs = []
        self._audio_refs = 0
        self._doa: Optional[float] = None
        self._doa_at = 0.0
        self._last_audio_emit = 0.0
        self.audio_channels = 4

    # -- video -----------------------------------------------------------------
    def acquire_video(self, src: str) -> None:
        topic, msg_type, conv, keep_depth = SOURCES[src]
        with self._lock:
            st = self._video.setdefault(src, {"sub": None, "refs": 0})
            st["refs"] += 1
            if st["sub"] is None:
                def cb(msg, _src=src, _conv=conv, _keep=keep_depth):
                    if _keep:
                        try:
                            self._depth[_src] = (convert.image_u16(msg), time.monotonic())
                        except Exception:
                            pass
                    jpeg = _conv(msg)
                    if jpeg is not None:
                        self._on_frame(_src, jpeg)
                qos = SENSOR_QOS if msg_type is CompressedImage else IMAGE_QOS
                st["sub"] = self.create_subscription(msg_type, topic, cb, qos)
                self.get_logger().info("video[%s]: subscribed to %s" % (src, topic))

    def release_video(self, src: str) -> None:
        with self._lock:
            st = self._video.get(src)
            if not st:
                return
            st["refs"] -= 1
            if st["refs"] <= 0 and st["sub"] is not None:
                self.destroy_subscription(st["sub"])
                st["sub"], st["refs"] = None, 0
                self._depth.pop(src, None)
                self.get_logger().info("video[%s]: unsubscribed (no viewers)" % src)

    def depth_at(self, src: str, fx: float, fy: float) -> Optional[float]:
        """Depth in mm at a normalised image position (0..1, 0..1)."""
        entry = self._depth.get(src)
        if entry is None or time.monotonic() - entry[1] > 2.0:
            return None
        arr = entry[0]
        u = int(min(max(fx, 0.0), 0.9999) * arr.shape[1])
        v = int(min(max(fy, 0.0), 0.9999) * arr.shape[0])
        return convert.depth_at(arr, u, v)

    # -- audio -----------------------------------------------------------------
    def acquire_audio(self) -> None:
        with self._lock:
            self._audio_refs += 1
            if not self._audio_subs:
                self._audio_subs = [
                    self.create_subscription(ByteMultiArray, "/kinect/audio/raw",
                                             self._on_raw, AUDIO_QOS),
                    self.create_subscription(Float32, "/kinect/audio/doa",
                                             self._on_doa, AUDIO_QOS),
                ]
                self.get_logger().info("audio: subscribed")

    def release_audio(self) -> None:
        with self._lock:
            self._audio_refs -= 1
            if self._audio_refs <= 0 and self._audio_subs:
                for s in self._audio_subs:
                    self.destroy_subscription(s)
                self._audio_subs, self._audio_refs = [], 0
                self.get_logger().info("audio: unsubscribed")

    def _on_doa(self, msg: Float32) -> None:
        self._doa = float(msg.data)
        self._doa_at = time.monotonic()

    def _on_raw(self, msg: ByteMultiArray) -> None:
        now = time.monotonic()
        if now - self._last_audio_emit < 0.08:      # ~12 Hz is plenty for a UI
            return
        self._last_audio_emit = now
        try:
            channels = msg.layout.dim[1].size if len(msg.layout.dim) > 1 else self.audio_channels
            levels = convert.channel_levels(msg.data, channels)
        except Exception:
            return
        doa = self._doa if now - self._doa_at < 1.5 else None
        self._on_audio({"levels": levels, "doa": None if doa is None else round(doa, 1)})

    # -- parameters on the publisher side -----------------------------------
    def set_doa_method(self, method: str) -> bool:
        """Switch the audio publisher's DOA estimator (srp | gcc)."""
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient
        client = AsyncParameterClient(self, "audio_publisher")
        if not client.wait_for_services(timeout_sec=2.0):
            return False
        client.set_parameters([Parameter("doa_method", Parameter.Type.STRING, method)])
        return True

    def stream_stats(self) -> dict:
        with self._lock:
            return {src: st["refs"] for src, st in self._video.items() if st["refs"] > 0}


def start(on_frame, on_audio) -> ConsoleBridge:
    """rclpy.init + spin the bridge in a daemon thread."""
    if not rclpy.ok():
        rclpy.init()
    node = ConsoleBridge(on_frame, on_audio)
    threading.Thread(target=rclpy.spin, args=(node,), name="rclpy-spin", daemon=True).start()
    return node
