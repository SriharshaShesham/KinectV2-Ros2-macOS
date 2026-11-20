#!/bin/bash
# Kinect v2 USB Connection Diagnostics

echo "=========================================="
echo "Kinect v2 USB Diagnostics"
echo "=========================================="
echo

echo "1. USB Device Tree - All connected devices:"
echo "----------------------------------------"
ioreg -p IOUSB -l | grep -E "USB Product Name|idVendor|idProduct" | paste - - - | head -20
echo

echo "2. Checking for Microsoft devices (Kinect Vendor ID: 0x045e):"
echo "----------------------------------------"
result=$(ioreg -p IOUSB -l | grep -i "microsoft\|045e")
if [ -z "$result" ]; then
    echo "❌ NO Microsoft/Kinect devices found!"
    echo "   The Kinect v2 is NOT detected on USB."
else
    echo "✓ Found: $result"
fi
echo

echo "3. USB3 Gen2 Hub port status:"
echo "----------------------------------------"
ioreg -p IOUSB -l | grep -A 30 "USB3 Gen2 Hub" | grep -E "locationID|sessionID|USB Product"
echo

echo "4. Kinect v2 Requirements:"
echo "----------------------------------------"
echo "   • Vendor ID: 0x045e (Microsoft)"
echo "   • Connection: USB 3.0 (5+ Gbps minimum)"
echo "   • Power: Device must be powered ON (LED lit)"
echo "   • Firmware: May need update on Windows if in bootloader"
echo

echo "5. What to try:"
echo "----------------------------------------"
echo "   a) Verify Kinect LED is illuminated (powered on)"
echo "   b) Try different USB 3.0 port (not USB 2.0)"
echo "   c) Unplug and replug the Kinect"
echo "   d) Run: ioreg -p IOUSB -l | grep -i kinect"
echo "   e) On Windows: Connect Kinect to update firmware"
echo
