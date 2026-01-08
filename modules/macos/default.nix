# macOS-specific modules
# Darwin configurations for ROS2 development
{
  config,
  lib,
  pkgs,
  ...
}:
{
  imports = [
    ./packages.nix
    ./homebrew.nix
    ./system.nix
    ./shell.nix
  ];
}
