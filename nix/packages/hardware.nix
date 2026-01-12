# Hardware interface packages for robotics
# Includes camera drivers, CAN bus tools, serial communication, and GPIO
#
# Usage:
#   packages.hardware - Full hardware interface stack
#   packages.hardwareCore - Minimal hardware tools
{ pkgs, ... }:

with pkgs; [
  # Camera drivers and utilities
  libv4l              # Video4Linux library
  v4l-utils           # V4L2 control utilities
  gphoto2             # Camera control tool

  # CAN bus tools
  can-utils           # SocketCAN utilities (candump, cansend, etc.)

  # Serial communication
  socat               # Multipurpose relay (serial/network)
  screen              # Terminal multiplexer (serial console)
  minicom             # Serial terminal emulator
  picocom             # Minimal serial terminal

  # USB utilities
  usbutils            # lsusb and friends
  libusb1             # USB library

  # GPIO and I2C (Linux only)
] ++ lib.optionals stdenv.isLinux [
  libgpiod            # GPIO character device library
  i2c-tools           # I2C/SMBus tools (i2cdetect, i2cget, etc.)
]

# Note: ROS2-specific hardware packages should be added via robostack/pixi:
#   - ros-humble-librealsense2
#   - ros-humble-realsense2-camera
#   - ros-humble-ros2-control
#   - ros-humble-ros2-controllers
#   - ros-humble-canopen-master
#   - ros-humble-canopen-402-driver
# These are not available in nixpkgs and should be installed via:
#   pixi add ros-humble-<package-name>
