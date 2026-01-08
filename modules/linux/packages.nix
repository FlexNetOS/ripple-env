# Linux-specific packages
# Additional packages needed on Linux systems
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
  home.packages = with pkgs; [
    # System utilities
    inotify-tools      # File system monitoring
    usbutils           # USB device tools (lsusb)
    pciutils           # PCI device tools (lspci)
    lm_sensors         # Hardware monitoring

    # Network tools
    iproute2           # Network management
    ethtool            # Network interface config
    bridge-utils       # Network bridging

    # Debug tools
    strace             # System call tracer
    ltrace             # Library call tracer
    gdb                # GNU debugger

    # Serial/CAN tools (common in robotics)
    minicom            # Serial terminal
    can-utils          # CAN bus utilities
    picocom            # Minimal serial terminal
  ];

  # Linux-specific environment variables
  home.sessionVariables = {
    # Allow core dumps for debugging
    MALLOC_CHECK_ = "3";
  };
}
