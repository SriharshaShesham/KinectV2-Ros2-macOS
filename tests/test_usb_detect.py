"""Kinect detection from macOS ioreg output (console + hotplug share the rules)."""
import plistlib

import kinect_hotplug as hp
from kinect_console import usb


def node(name, vid, pid, speed, serial="", children=()):
    n = {"USB Product Name": name, "idVendor": vid, "idProduct": pid, "Device Speed": speed}
    if serial:
        n["USB Serial Number"] = serial
    if children:
        n["IORegistryEntryChildren"] = list(children)
    return n


def tree(*devices):
    hub = node("USB3 Gen2 Hub", 0x05AC, 0x800C, 4, children=devices)
    return plistlib.dumps({"IORegistryEntryChildren": [{"IORegistryEntryChildren": [
        hub, node("USB Receiver", 0x046D, 0xC52B, 1)]}]})


KINECT = node("Xbox NUI Sensor", 0x045E, 0x02C4, 3, "292776234347")
KINECT_USB2 = node("Xbox NUI Sensor", 0x045E, 0x02C4, 2, "292776234347")


def test_console_finds_kinect_nested_under_hub():
    st = usb.find_kinect(usb.parse_ioreg(tree(KINECT)))
    assert st == {"present": True, "superspeed": True, "speed": "5 Gb/s", "serial": "292776234347"}


def test_console_flags_usb2_and_absent():
    assert usb.find_kinect(usb.parse_ioreg(tree(KINECT_USB2)))["superspeed"] is False
    assert usb.find_kinect(usb.parse_ioreg(tree()))["present"] is False
    assert usb.parse_ioreg(b"") == []


def test_other_microsoft_device_is_not_a_kinect():
    mouse = node("Microsoft Mouse", 0x045E, 0x07FD, 1)
    assert usb.find_kinect(usb.parse_ioreg(tree(mouse)))["present"] is False


def flat(*devices):
    return [dict(vid=d["idVendor"], name=d["USB Product Name"], speed=d["Device Speed"]) for d in devices]


def test_hotplug_match_detect():
    rule = hp.KINECT_USB
    assert hp.match_detect(rule, flat(KINECT))[:2] == (True, True)
    assert hp.match_detect(rule, flat(KINECT_USB2))[:2] == (True, False)
    assert hp.match_detect(rule, [])[:2] == (False, False)
    assert hp.match_detect(None, [])[:2] == (True, True)


def test_real_ioreg_parses():
    devs = usb.parse_ioreg(__import__("subprocess").run(
        ["ioreg", "-a", "-l", "-p", "IOUSB", "-w0"], capture_output=True).stdout)
    assert isinstance(devs, list)
