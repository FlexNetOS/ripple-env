# Common modules for all platforms
# Aggregates shared configurations for ROS2 development
{ pkgs, lib, ... }:
{
  imports = [
    ./direnv.nix
    ./git.nix
    ./packages.nix
    ./nix
    ./editor
    ./shell
  ];
}
