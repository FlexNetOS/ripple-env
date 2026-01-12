# Multi-distro shell aggregator
# Imports all ROS2 distribution-specific shells
#
# Usage in flake.nix:
#   multiDistro = import ./nix/shells/multi-distro { ... };
#   devShells.humble = multiDistro.humble;
#   devShells.iron = multiDistro.iron;
#   devShells.rolling = multiDistro.rolling;
{ pkgs, lib, system, packages, commands, colconDefaults, ... }:

let
  commonArgs = {
    inherit pkgs lib system packages commands colconDefaults;
  };
in
{
  # LTS release - recommended for production
  humble = import ./humble.nix commonArgs;

  # Latest stable release
  iron = import ./iron.nix commonArgs;

  # Development branch - not for production
  rolling = import ./rolling.nix commonArgs;
}
