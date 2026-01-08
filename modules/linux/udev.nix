# udev rules configuration
# Hardware device access for ROS2/robotics
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  # Note: udev rules need to be installed system-wide (NixOS config)
  # This file provides templates and documentation

  # For NixOS systems, add to configuration.nix:
  # services.udev.extraRules = ''
  #   # Generic USB serial adapters
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
  #
  #   # FTDI devices
  #   SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", MODE="0666", GROUP="plugdev"
  #
  #   # CH340/CH341 USB-Serial
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
  #
  #   # CP210x USB-Serial
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
  #
  #   # Arduino boards
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"
  #
  #   # Teensy boards
  #   SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", MODE="0666", GROUP="plugdev"
  #
  #   # RealSense cameras
  #   SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
  #
  #   # Xbox controllers (for teleop)
  #   SUBSYSTEM=="usb", ATTRS{idVendor}=="045e", MODE="0666", GROUP="plugdev"
  #
  #   # Dynamixel servos
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout", SYMLINK+="dynamixel"
  #
  #   # RPLidar
  #   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar"
  # '';

  # Create a helper script to generate udev rules
  home.packages = with pkgs; [
    # udev info tools
    usbutils
  ];

  # Add a script to identify USB devices
  home.file.".local/bin/ros2-udev-helper" = {
    executable = true;
    text = ''
      #!/usr/bin/env bash
      # Helper script to identify USB devices and generate udev rules

      echo "Connected USB devices:"
      echo "======================"
      lsusb

      echo ""
      echo "To create a udev rule for a device, note the ID (xxxx:yyyy)"
      echo "Then add to /etc/udev/rules.d/99-ros2-devices.rules:"
      echo ""
      echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", MODE="0666", GROUP="plugdev"'
      echo ""
      echo "For serial devices, use:"
      echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", MODE="0666", GROUP="dialout"'
      echo ""
      echo "After adding rules, run: sudo udevadm control --reload-rules && sudo udevadm trigger"
    '';
  };
}
