# Nushell configuration
# Modern shell with structured data support
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
  programs.nushell = {
    enable = mkDefault true;

    # Configuration
    extraConfig = ''
      # General settings
      $env.config = {
        show_banner: false
        table: {
          mode: rounded
          index_mode: auto
        }
        completions: {
          case_sensitive: false
          quick: true
          partial: true
          algorithm: "fuzzy"
        }
        history: {
          max_size: 100000
          sync_on_enter: true
          file_format: "sqlite"
        }
        filesize: {
          metric: true
          format: "auto"
        }
        cursor_shape: {
          emacs: line
          vi_insert: line
          vi_normal: block
        }
        edit_mode: emacs
        shell_integration: true
        use_kitty_protocol: true
      }

      # ROS2 environment helpers
      def ros2-env [] {
        print "ROS2 Environment Variables:"
        env | where name =~ "ROS" | table
      }

      def ros2-nodes [] {
        ^ros2 node list
      }

      def ros2-topics [] {
        ^ros2 topic list
      }

      def ros2-services [] {
        ^ros2 service list
      }

      # Colcon build helper
      def colcon-build [...args] {
        ^colcon build --symlink-install ...$args
      }

      # Quick directory navigation
      def --env z [...args] {
        let result = (zoxide query ...$args)
        cd $result
      }
    '';

    # Environment configuration
    extraEnv = ''
      # PATH additions
      $env.PATH = ($env.PATH | split row (char esep) | prepend $"($env.HOME)/.local/bin")

      # Editor
      $env.EDITOR = "hx"
      $env.VISUAL = "hx"

      # ROS2 defaults
      $env.ROS_DOMAIN_ID = ($env.ROS_DOMAIN_ID? | default "0")

      # Less pager settings
      $env.LESS = "-R -F -X"

      # LS Colors using vivid
      if (which vivid | is-not-empty) {
        $env.LS_COLORS = (vivid generate gruvbox-dark)
      }
    '';

    # Shell aliases (nushell syntax)
    shellAliases = {
      # Better defaults
      "ll" = "ls -la";
      "la" = "ls -a";

      # Navigation
      ".." = "cd ..";
      "..." = "cd ../..";
      "cd-" = "cd -";

      # Git shortcuts
      "gs" = "git status";
      "ga" = "git add";
      "gc" = "git commit";
      "gp" = "git push";
      "gl" = "git pull";
      "gd" = "git diff";
      "gco" = "git checkout";
      "gb" = "git branch";

      # ROS2 shortcuts
      "cb" = "colcon build";
      "cbs" = "colcon build --symlink-install";
      "ct" = "colcon test";

      # Nix shortcuts
      "nrs" = "nom develop";
    };
  };
}
