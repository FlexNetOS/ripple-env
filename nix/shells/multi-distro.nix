# Multi-distro ROS2 shell definitions
# Supports switching between ROS2 Humble, Iron, and Rolling
#
# Usage in flake.nix:
#   multiDistro = import ./nix/shells/multi-distro.nix { ... };
#   devShells.humble = multiDistro.humble;
#   devShells.iron = multiDistro.iron;
#   devShells.rolling = multiDistro.rolling;
{ pkgs, lib, system, packages, commands, common, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;

  # Create a ROS2 shell for a specific distro
  mkRosShell = distro: description: pkgs.mkShell {
    name = "ros2-${distro}";

    packages = packages.defaultShell ++ commands.defaultShell;

    inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

    # Set ROS2 distro
    ROS_DISTRO = distro;

    shellHook = common.baseShellHook + ''
      echo ""
      echo "ROS2 ${distro} Development Environment"
      echo "========================================"
      echo "  Distro: ${distro}"
      echo "  Description: ${description}"
      echo ""
      echo "Note: ROS2 packages are managed via Pixi/RoboStack"
      echo "  To switch distros in pixi.toml, update the channel:"
      echo "    channels = [\"robostack-${distro}\", ...]"
      echo ""

      # Set ROS2 distro environment
      export ROS_DISTRO="${distro}"

      # Check if pixi is configured for this distro
      if [[ -f "pixi.toml" ]]; then
        if ! grep -q "robostack-${distro}" pixi.toml 2>/dev/null; then
          echo "Warning: pixi.toml may not be configured for ${distro}"
          echo "  Current robostack channel may differ from selected distro"
          echo ""
        fi
      fi
    '';
  };
in
{
  # ROS2 Humble - LTS until May 2027
  humble = mkRosShell "humble" "LTS release (May 2022 - May 2027)";

  # ROS2 Iron - Standard support (released May 2023)
  iron = mkRosShell "iron" "Standard release (May 2023 - Nov 2024)";

  # ROS2 Rolling - Development branch (always latest)
  rolling = mkRosShell "rolling" "Development branch (always latest)";
}
