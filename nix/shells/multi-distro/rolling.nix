# ROS2 Rolling Development Shell (Development Branch)
# Bleeding edge with latest features - NOT for production.
#
# Usage:
#   nix develop .#rolling
#
# Features:
#   - Latest development features
#   - Unstable API (may change)
#   - Good for testing upcoming features
{ pkgs, lib, system, packages, commands, colconDefaults, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;
  inherit (pkgs.lib) optionalString;
in
pkgs.mkShell {
  name = "ros2-rolling";

  packages = packages.defaultShell ++ commands.defaultShell;

  COLCON_DEFAULTS_FILE = toString colconDefaults;
  EDITOR = "hx";
  VISUAL = "hx";

  # ROS2 distro identification
  ROS_DISTRO = "rolling";
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
      # Note: Would need pixi.toml configured for Rolling channel
      eval "$(pixi shell-hook 2>/dev/null)" || true
    fi

    # Only show banner in interactive shells
    if [[ $- == *i* ]]; then
      echo ""
      echo "⚠️  ROS2 Rolling Development Environment"
      echo "========================================="
      echo "  Distro: Rolling Ridley (Development)"
      echo "  Support: Continuous updates"
      echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
      echo ""
      echo "WARNING: Rolling is a development branch!"
      echo "  - API may change without notice"
      echo "  - Not recommended for production"
      echo "  - Use Humble (LTS) for stable development"
      echo ""
      echo "Rolling is useful for:"
      echo "  - Testing upcoming features"
      echo "  - Contributing to ROS2 core"
      echo "  - Early adoption experiments"
      echo ""
    fi
  '';
}
