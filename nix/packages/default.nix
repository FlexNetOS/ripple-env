# Package aggregator - imports all package modules
# Usage in flake.nix:
#   packages = import ./nix/packages { inherit pkgs lib; };
#   then use: packages.base, packages.devTools, packages.holochain, etc.
#
# Performance optimization: Use feature flags for heavy dependencies
#   packages.devToolsMinimal = import ./dev-tools.nix { inherit pkgs; withKubernetes = false; withHeavyVMs = false; withAI = false; };
{ pkgs, lib, ... }:

let
  platform = import ./platform.nix { inherit pkgs lib; };
in
{
  # Minimal packages for fastest startup (CI, scripting)
  minimal = import ./minimal.nix { inherit pkgs; };

  # Base packages for fast shell startup
  base = import ./base.nix { inherit pkgs; };

  # Full development tools (optional extras)
  devTools = import ./dev-tools.nix { inherit pkgs; };

  # Development tools with feature flags for optimization
  devToolsMinimal = import ./dev-tools.nix {
    inherit pkgs;
    withKubernetes = false;
    withHeavyVMs = false;
    withAI = false;
  };

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

  # Convenience: minimal packages for fastest shell
  minimalShell = (import ./minimal.nix { inherit pkgs; });

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
