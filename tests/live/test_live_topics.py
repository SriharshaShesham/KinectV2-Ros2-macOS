"""Live checks against the running publishers. Needs:
    ros2 launch kinect kinect.launch.py      (in another terminal)
    KINECT_LIVE=1 pytest tests/live -v
Several independent subscriber nodes run at once to prove fan-out."""
import threading
import time

import pytest

pytestmark = pytest.mark.live

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2  # noqa: E402
from std_msgs.msg import ByteMultiArray  # noqa: E402

REL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
TOPICS = {   # topic: (type, minimum Hz)
    "/kinect/rgb/compressed": (CompressedImage, 8.0),
    "/kinect/depth": (Image, 4.0),
    "/kinect/ir": (Image, 8.0),
    "/kinect/rgb/registered": (Image, 5.0),
    "/kinect/depth/color_aligned": (Image, 3.0),
    "/camera/depth/camera_info": (CameraInfo, 5.0),
    "/kinect/audio/raw": (ByteMultiArray, 15.0),
    "/kinect/pointcloud": (PointCloud2, 2.0),
}


class Counter(Node):
    def __init__(self, idx, topics):
        super().__init__("kinect_live_test_%d" % idx)
        self.msgs = {t: [] for t in topics}
        for t in topics:
            typ = TOPICS[t][0]
            self.create_subscription(typ, t, lambda m, t=t: self.msgs[t].append(m), REL)


@pytest.fixture(scope="module")
def counters():
    rclpy.init()
    nodes = [Counter(i, list(TOPICS)) for i in range(3)]   # three independent subscribers
    ex = MultiThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)                     # discovery + driver warm-up
    for n in nodes:
        for v in n.msgs.values():
            v.clear()
    time.sleep(6.0)
    yield nodes
    ex.shutdown()
    rclpy.shutdown()


@pytest.mark.parametrize("topic", list(TOPICS))
def test_every_subscriber_gets_the_topic(counters, topic):
    for n in counters:
        hz = len(n.msgs[topic]) / 6.0
        assert hz >= TOPICS[topic][1], "%s: %s at %.1f Hz" % (n.get_name(), topic, hz)


def test_depth_is_millimetres_with_valid_pixels(counters):
    import numpy as np
    m = counters[0].msgs["/kinect/depth"][-1]
    assert (m.width, m.height, m.encoding) == (512, 424, "mono16")
    d = np.frombuffer(bytes(m.data), "<u2")
    valid = d[d > 0]
    assert valid.size > 0.2 * d.size and 300 < np.median(valid) < 8000


def test_camera_info_is_kinect_calibration(counters):
    k = counters[0].msgs["/camera/depth/camera_info"][-1].k
    assert 340 < k[0] < 390 and 230 < k[2] < 280


def test_audio_is_four_channel_int16(counters):
    m = counters[0].msgs["/kinect/audio/raw"][-1]
    dims = {d.label: d.size for d in m.layout.dim}
    assert dims["channels"] == 4 and len(m.data) == dims["frames"] * 4 * 2
