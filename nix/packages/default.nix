# Package aggregator - imports all package modules
# Usage in flake.nix:
#   packages = import ./nix/packages { inherit pkgs lib; };
#   then use: packages.base, packages.devTools, packages.holochain, etc.
{ pkgs, lib, ... }:

let
  platform = import ./platform.nix { inherit pkgs lib; };
in
{
  # Base packages for fast shell startup
  base = import ./base.nix { inherit pkgs; };

  # Full development tools (optional extras)
  devTools = import ./dev-tools.nix { inherit pkgs; };

  # Holochain P2P packages (requires overlay applied to pkgs)
  holochain = import ./holochain.nix { inherit pkgs; };

  # Hardware interface packages (cameras, CAN, serial, GPIO)
  hardware = import ./hardware.nix { inherit pkgs lib; };

  # Simulation packages (visualization, physics, 3D rendering)
  simulation = import ./simulation.nix { inherit pkgs lib; };

  # Platform-specific packages
  linux = platform.linux;
  darwin = platform.darwin;
  platformAll = platform.all;

  # Convenience: all packages combined for default shell
  defaultShell = (import ./base.nix { inherit pkgs; })
    ++ (import ./holochain.nix { inherit pkgs; })
    ++ platform.all;

  # Convenience: all packages combined for full shell
  fullShell = (import ./base.nix { inherit pkgs; })
    ++ (import ./dev-tools.nix { inherit pkgs; })
    ++ (import ./holochain.nix { inherit pkgs; })
    ++ platform.all;

  # Robotics shell: includes hardware and simulation packages
  roboticsShell = (import ./base.nix { inherit pkgs; })
    ++ (import ./hardware.nix { inherit pkgs lib; })
    ++ (import ./simulation.nix { inherit pkgs lib; })
    ++ platform.all;
}
