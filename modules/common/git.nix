# Git configuration
# Standard git setup for ROS2 development
# Based on GustavoWidman/nix patterns
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
  programs.git = {
    enable = mkDefault true;

    # Enable LFS for large files (common in robotics)
    lfs.enable = mkDefault true;

    # Default branch name
    extraConfig = {
      init.defaultBranch = mkDefault "main";

      # Merge strategy
      merge.conflictStyle = mkDefault "zdiff3";

      # Pull behavior - rebase by default
      pull = {
        rebase = mkDefault true;
      };

      # Rebase settings
      rebase = {
        autoStash = mkDefault true;
        autoSquash = mkDefault true;
      };

      # Push behavior
      push = {
        autoSetupRemote = mkDefault true;
        default = mkDefault "current";
      };

      # Core settings
      core = {
        autocrlf = mkDefault "input";
        editor = mkDefault "hx"; # Helix editor
      };

      # Diff settings
      diff = {
        algorithm = mkDefault "histogram";
        colorMoved = mkDefault "default";
      };

      # Credential helper
      credential.helper = mkDefault "cache --timeout=3600";

      # GitHub CLI integration
      url = {
        "ssh://git@github.com/" = {
          insteadOf = "https://github.com/";
        };
      };
    };

    # Global gitignore patterns
    ignores = [
      # OS files
      ".DS_Store"
      "Thumbs.db"

      # IDE/Editor
      ".idea/"
      ".vscode/"
      "*.swp"
      "*.swo"
      "*~"
      ".helix/"

      # Nix
      "result"
      "result-*"
      ".direnv/"

      # Python
      "__pycache__/"
      "*.pyc"
      "*.pyo"
      ".python-version"
      "*.egg-info/"
      ".eggs/"
      "venv/"
      ".venv/"

      # Rust
      "target/"
      "Cargo.lock"

      # ROS2 / Colcon
      "build/"
      "install/"
      "log/"
      ".colcon/"

      # pixi
      ".pixi/"

      # Environment
      ".env"
      ".env.local"
      "*.env"
    ];
  };
}
