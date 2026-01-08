# Linux user configuration
# User groups and permissions for ROS2 development
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
  # Note: This is for NixOS systems
  # Uncomment and customize for your setup
  #
  # users.users.${username} = {
  #   isNormalUser = true;
  #   extraGroups = [
  #     "wheel"          # sudo access
  #     "networkmanager" # Network management
  #     "dialout"        # Serial port access (important for robotics)
  #     "plugdev"        # USB device access
  #     "video"          # Video devices
  #     "docker"         # Docker access
  #     "gpio"           # GPIO access (Raspberry Pi, etc.)
  #     "i2c"            # I2C bus access
  #     "spi"            # SPI bus access
  #   ];
  # };

  # For home-manager standalone, we just configure dialout group hint
  home.sessionVariables = {
    # Hint for serial port access
    # Run: sudo usermod -aG dialout $USER
    ROS2_SERIAL_HINT = "If serial access fails, run: sudo usermod -aG dialout $USER";
  };
}
