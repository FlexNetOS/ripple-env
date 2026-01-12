# ROS2 Iron Development Shell (May 2023 - Nov 2024)
# Standard release with latest stable features.
#
# Usage:
#   nix develop .#iron
#
# Features:
#   - Latest stable features
#   - Improved type adapters
#   - Enhanced service introspection
{ pkgs, lib, system, packages, commands, colconDefaults, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;
  inherit (pkgs.lib) optionalString;
in
pkgs.mkShell {
  name = "ros2-iron";

  packages = packages.defaultShell ++ commands.defaultShell;

  COLCON_DEFAULTS_FILE = toString colconDefaults;
  EDITOR = "hx";
  VISUAL = "hx";

  # ROS2 distro identification
  ROS_DISTRO = "iron";
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
      # Note: Would need pixi.toml configured for Iron channel
      eval "$(pixi shell-hook 2>/dev/null)" || true
    fi

    # Only show banner in interactive shells
    if [[ $- == *i* ]]; then
      echo ""
      echo "ROS2 Iron Development Environment"
      echo "=================================="
      echo "  Distro: Iron Irwini"
      echo "  Support: May 2023 - Nov 2024"
      echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
      echo ""
      echo "Note: Iron is a standard release. For production, consider Humble (LTS)."
      echo ""
      echo "Iron-specific features:"
      echo "  - Improved type adaptation"
      echo "  - Service introspection"
      echo "  - Enhanced lifecycle management"
      echo ""
    fi
  '';
}
