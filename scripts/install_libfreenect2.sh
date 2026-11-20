#!/usr/bin/env bash
# Build and install libfreenect2 (the Kinect v2 driver library) on macOS.
#
# Reproduces the build this repo was tested with:
#   libfreenect2 commit fd64c5d (upstream master, 2020-03-01)
#   Homebrew cmake 4.2.1, libusb 1.0.29, glfw 3.4, jpeg-turbo 3.1
#   Apple Silicon, macOS 26; installed to /usr/local
#
# There is no Homebrew formula or conda package for libfreenect2 on Apple
# Silicon, so it is built from source. Usage:
#   bash scripts/install_libfreenect2.sh            # clone to ~/src/libfreenect2
#   SRC=~/Documents/libfreenect2 bash scripts/install_libfreenect2.sh
set -euo pipefail

SRC="${SRC:-$HOME/src/libfreenect2}"
COMMIT=fd64c5d9b214df6f6a55b4419357e51083f15d93
PREFIX="${PREFIX:-/usr/local}"

echo "==> Homebrew dependencies"
brew install cmake pkgconf libusb glfw jpeg-turbo

if [ ! -d "$SRC/.git" ]; then
  echo "==> Cloning libfreenect2 into $SRC"
  git clone https://github.com/OpenKinect/libfreenect2.git "$SRC"
fi
git -C "$SRC" checkout -q "$COMMIT"

echo "==> Configuring"
mkdir -p "$SRC/build"
# CMAKE_POLICY_VERSION_MINIMUM: libfreenect2 declares cmake_minimum_required(2.8),
# which CMake 4 refuses without this. OpenCL (depth decoding on the Apple GPU),
# OpenGL (GLFW) and VideoToolbox (hardware JPEG decode for color) are found
# automatically on macOS.
cmake -S "$SRC" -B "$SRC/build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DENABLE_OPENCL=ON -DENABLE_OPENGL=ON -DENABLE_CUDA=OFF \
  -DBUILD_EXAMPLES=ON -DBUILD_OPENNI2_DRIVER=OFF

echo "==> Building"
cmake --build "$SRC/build" -j "$(sysctl -n hw.ncpu)"

echo "==> Installing to $PREFIX (needs sudo when $PREFIX is root-owned)"
if [ -w "$PREFIX/lib" ]; then cmake --install "$SRC/build"; else sudo cmake --install "$SRC/build"; fi

echo "==> Check"
PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig" pkg-config --modversion freenect2
grep -E "WITH_(OPENCL|OPENGL|VT)_SUPPORT" "$PREFIX/include/libfreenect2/config.h"
echo
echo "Done. With the Kinect plugged in, try the bundled viewer:"
echo "  $SRC/build/bin/Protonect cl"
