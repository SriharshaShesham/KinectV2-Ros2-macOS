import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("src/kinect_tools", "src/kinect_console", "hotplug"):
    sys.path.insert(0, os.path.join(ROOT, p))


def pytest_configure(config):
    config.addinivalue_line("markers", "live: needs the Kinect publishers running "
                            "(ros2 launch kinect kinect.launch.py); skipped otherwise")


def pytest_collection_modifyitems(config, items):
    if os.environ.get("KINECT_LIVE") == "1":
        return
    skip = pytest.mark.skip(reason="live test: start the driver and run with KINECT_LIVE=1")
    for item in items:
        if "live" in item.keywords:
            item.add_marker(skip)
