# direnv configuration
# Enhanced direnv setup for automatic environment activation
# Based on GustavoWidman/nix and RGBCube/ncc patterns
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  # direnv program configuration for home-manager
  programs.direnv = {
    enable = mkDefault true;

    # nix-direnv provides faster direnv loading for nix environments
    nix-direnv.enable = mkDefault true;

    # Silent mode - reduces shell noise
    silent = mkDefault true;

    # direnv configuration
    config = {
      # Whitelist directories for automatic loading
      whitelist = {
        # Add prefixes that are safe to auto-load
        prefix = [ ];
      };

      # Global settings
      global = {
        # Increase timeout for large nix builds
        warn_timeout = "15m";

        # Hide direnv loading messages for cleaner output
        hide_env_diff = true;
      };
    };

    # Standard library extensions for .envrc files
    stdlib = ''
      # Enhanced use_flake that handles pixi integration
      use_flake_pixi() {
        use flake
        if [[ -f pixi.toml ]]; then
          eval "$(pixi shell-hook)"
        fi
      }

      # Layout for ROS2 workspaces
      layout_ros2() {
        export ROS_DOMAIN_ID=''${ROS_DOMAIN_ID:-0}
        export ROS_LOCALHOST_ONLY=''${ROS_LOCALHOST_ONLY:-0}

        # Source local setup if exists
        if [[ -f install/local_setup.bash ]]; then
          source install/local_setup.bash
        fi
      }

      # Layout for colcon workspaces
      layout_colcon() {
        layout_ros2
        export COLCON_HOME="$PWD/.colcon"
        mkdir -p "$COLCON_HOME"
      }
    '';
  };
}
