# Common modules for all platforms
# Aggregates shared configurations for ROS2 development
{ pkgs, lib, ... }:
{
  imports = [
    ./ai
    ./direnv.nix
    ./git.nix
    ./packages.nix
    ./xdg.nix
    ./nix
    ./editor
    ./editor/neovim.nix
    ./shell
  ];
}
