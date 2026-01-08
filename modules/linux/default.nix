# Linux-specific modules
# NixOS configurations for ROS2 development
{
  config,
  lib,
  pkgs,
  ...
}:
{
  imports = [
    ./packages.nix
    ./users.nix
    ./docker.nix
    ./udev.nix
    ./systemd.nix
  ];
}
