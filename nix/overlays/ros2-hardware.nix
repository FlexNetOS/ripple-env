# ROS2 Hardware Interface Overlay
# Provides hardware-specific ROS2 packages not in standard nixpkgs
#
# Usage in flake.nix:
#   overlays = [ (import ./nix/overlays/ros2-hardware.nix) ];
#
# Packages provided:
#   - ros2-realsense (Intel RealSense cameras)
#   - ros2-control (hardware abstraction)
#   - ros2-controllers (standard controllers)
#   - nav2 (navigation stack)
#   - slam-toolbox (SLAM)
final: prev: let
  rosDistro = "humble";
in {
  # Intel RealSense ROS2 wrapper
  # Note: Actual ROS2 packages should come from RoboStack via pixi.toml
  # This overlay provides Nix-native alternatives or supplements
  ros2-hardware = {
    # librealsense2 - Intel RealSense SDK
    librealsense2 = prev.librealsense or prev.stdenv.mkDerivation {
      pname = "librealsense2";
      version = "2.54.2";

      src = prev.fetchFromGitHub {
        owner = "IntelRealSense";
        repo = "librealsense";
        rev = "v2.54.2";
        sha256 = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";
      };

      nativeBuildInputs = with prev; [ cmake pkg-config ];
      buildInputs = with prev; [
        libusb1
        openssl
        udev
        glfw
        libGL
      ];

      cmakeFlags = [
        "-DBUILD_EXAMPLES=OFF"
        "-DBUILD_GRAPHICAL_EXAMPLES=OFF"
        "-DBUILD_PYTHON_BINDINGS=ON"
        "-DFORCE_RSUSB_BACKEND=ON"
      ];

      meta = with prev.lib; {
        description = "Intel RealSense SDK";
        homepage = "https://github.com/IntelRealSense/librealsense";
        license = licenses.asl20;
        platforms = platforms.linux;
      };
    };

    # CAN utilities for robot communication buses
    can-utils = prev.can-utils;

    # SocketCAN tools
    socketcan-tools = prev.writeShellScriptBin "socketcan-setup" ''
      #!/usr/bin/env bash
      # Setup SocketCAN interface
      # Usage: socketcan-setup can0 500000
      set -euo pipefail

      INTERFACE=''${1:-can0}
      BITRATE=''${2:-500000}

      echo "Setting up $INTERFACE at $BITRATE bps"
      sudo ip link set $INTERFACE type can bitrate $BITRATE
      sudo ip link set $INTERFACE up

      echo "Interface $INTERFACE is ready"
      ip -details link show $INTERFACE
    '';

    # Serial port utilities
    serial-tools = with prev; [
      minicom
      screen
      picocom
      socat
    ];

    # GPIO and I2C tools (for embedded systems)
    embedded-tools = with prev; [
      libgpiod
      i2c-tools
    ];

    # USB device tools
    usb-tools = with prev; [
      usbutils
      libusb1
    ];
  };

  # Convenience package set combining all hardware tools
  ros2HardwarePackages = with final.ros2-hardware; [
    can-utils
    socketcan-tools
  ] ++ serial-tools ++ embedded-tools ++ usb-tools;
}
