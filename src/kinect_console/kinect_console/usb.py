"""Is the Kinect v2 on the macOS USB bus? (Same detection as hotplug/.)

Reads the I/O Registry with `ioreg -a -l -p IOUSB`; needs no permissions and
does not touch the device, so it works while the driver is streaming.
"""

import plistlib
import subprocess

KINECT_VENDOR_ID = 0x045E           # Microsoft
KINECT_PRODUCT_NAME = "Xbox NUI Sensor"
SUPERSPEED = 3                      # IOUSB "Device Speed": 3 = 5 Gb/s, 4 = 10 Gb/s
SPEED_NAMES = {0: "1.5 Mb/s", 1: "12 Mb/s", 2: "480 Mb/s", 3: "5 Gb/s", 4: "10 Gb/s"}


def parse_ioreg(data: bytes):
    devices = []

    def walk(node):
        if isinstance(node, dict):
            if "idVendor" in node:
                devices.append(dict(vid=node.get("idVendor", 0), pid=node.get("idProduct", 0),
                                    name=node.get("USB Product Name", ""),
                                    speed=node.get("Device Speed", 0),
                                    serial=node.get("USB Serial Number", "")))
            for child in node.get("IORegistryEntryChildren", []):
                walk(child)
        elif isinstance(node, list):
            for child in node:
                walk(child)

    if data and data.strip():
        walk(plistlib.loads(data))
    return devices


def find_kinect(devices) -> dict:
    for d in devices:
        if d["vid"] == KINECT_VENDOR_ID and d["name"] == KINECT_PRODUCT_NAME:
            return {"present": True, "superspeed": d["speed"] >= SUPERSPEED,
                    "speed": SPEED_NAMES.get(d["speed"], str(d["speed"])), "serial": d["serial"]}
    return {"present": False, "superspeed": False, "speed": None, "serial": None}


def kinect_status() -> dict:
    out = subprocess.run(["ioreg", "-a", "-l", "-p", "IOUSB", "-w0"],
                         capture_output=True, timeout=10).stdout
    return find_kinect(parse_ioreg(out))
