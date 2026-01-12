# ROS2 Humble Development Shell (LTS: May 2022 - May 2027)
# This is the recommended shell for production robotics development.
#
# Usage:
#   nix develop .#humble
#
# Features:
#   - Long-term support until May 2027
#   - Stable API with security updates
#   - Widest hardware/package compatibility
{ pkgs, lib, system, packages, commands, colconDefaults, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;
  inherit (pkgs.lib) optionalString;
in
pkgs.mkShell {
  name = "ros2-humble";

  packages = packages.defaultShell ++ commands.defaultShell;

  COLCON_DEFAULTS_FILE = toString colconDefaults;
  EDITOR = "hx";
  VISUAL = "hx";

  # ROS2 distro identification
  ROS_DISTRO = "humble";
  ROS_VERSION = "2";
  ROS_PYTHON_VERSION = "3";

  shellHook = ''
    # Ensure TMPDIR is valid
    export TMPDIR=''${TMPDIR:-/tmp}
    [ -d "$TMPDIR" ] || export TMPDIR=/tmp
    mkdir -p "$TMPDIR" 2>/dev/null || true

    # Define stub functions for RoboStack activation
    noa_add_path() { :; }
    export -f noa_add_path 2>/dev/null || true

    # Initialize pixi environment
    if [ -f pixi.toml ]; then
      ${optionalString isDarwin ''
        export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
      ''}
      eval "$(pixi shell-hook 2>/dev/null)" || true
    fi

    # Only show banner in interactive shells
    if [[ $- == *i* ]]; then
      echo ""
      echo "ROS2 Humble Development Environment (LTS)"
      echo "=========================================="
      echo "  Distro: Humble Hawksbill"
      echo "  Support: May 2022 - May 2027 (LTS)"
      echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
      echo ""
      echo "Quick start:"
      echo "  ros2 run demo_nodes_cpp talker     # C++ publisher"
      echo "  ros2 run demo_nodes_py listener   # Python subscriber"
      echo "  ros2 topic list                    # List topics"
      echo ""
    fi
  '';
}
