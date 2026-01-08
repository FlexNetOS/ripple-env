# System builder utilities
# Provides helper functions to construct NixOS/Darwin systems
{
  lib,
  inputs,
  ...
}:
let
  inherit (lib) optionalString;
in
{
  # Default system settings for ROS2 development
  defaultSystemSettings = {
    isDev = true;
    isDesktop = false;
    ros2 = {
      enabled = true;
      distro = "humble";
    };
  };

  # Get platform-specific module path
  getPlatformModules =
    { isDarwin, isLinux }:
    if isDarwin then
      [ ../modules/macos ]
    else if isLinux then
      [ ../modules/linux ]
    else
      [ ];

  # Common modules for all systems
  commonModules = [ ../modules/common ];
}
