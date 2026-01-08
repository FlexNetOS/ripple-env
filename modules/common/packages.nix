# Common packages for all platforms
# Core development tools for ROS2 environment
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault optionals;
  inherit (pkgs.stdenv) isDarwin isLinux;
in
{
  # Core packages available in the development shell
  home.packages = with pkgs; [
    # Nix tools
    nix-output-monitor # nom - better nix output
    nix-tree           # Visualize nix derivations
    nixfmt-rfc-style   # Nix formatter

    # Shell utilities
    bat                # Better cat
    eza                # Better ls
    fd                 # Better find
    ripgrep            # Better grep
    fzf                # Fuzzy finder
    jq                 # JSON processor
    yq                 # YAML processor

    # System monitoring
    btop               # Better top
    htop               # Interactive process viewer

    # Development tools
    git                # Version control
    gh                 # GitHub CLI

    # Directory navigation
    zoxide             # Smart cd

    # Network tools
    curl               # HTTP client
    wget               # Download tool

    # Archive tools
    unzip
    zip
    gzip

    # Text processing
    gnused
    gawk

    # Build tools (supplementary to pixi)
    pkg-config

    # AI assistants
    aichat             # Provider-agnostic AI CLI
  ] ++ optionals isLinux [
    # Linux-specific packages
    inotify-tools      # File watching
    usbutils           # USB utilities
  ] ++ optionals isDarwin [
    # macOS-specific packages
    coreutils          # GNU coreutils
  ];

  # Shell aliases for common operations
  home.shellAliases = {
    # Nix shortcuts
    "nrs" = "nom develop";
    "nrb" = "nom build";

    # Better defaults
    "ls" = "eza --icons";
    "ll" = "eza -la --icons";
    "la" = "eza -a --icons";
    "lt" = "eza --tree --icons";
    "cat" = "bat";
    "grep" = "rg";
    "find" = "fd";

    # ROS2 shortcuts
    "cb" = "colcon build";
    "cbs" = "colcon build --symlink-install";
    "ct" = "colcon test";
    "ctr" = "colcon test-result --verbose";

    # Git shortcuts
    "gs" = "git status";
    "ga" = "git add";
    "gc" = "git commit";
    "gp" = "git push";
    "gl" = "git pull";
    "gd" = "git diff";
    "gco" = "git checkout";
    "gb" = "git branch";
    "glog" = "git log --oneline --graph --decorate";

    # AI assistants
    "ai" = "aichat";
    "ai-code" = "aichat --role coder";
    "ai-explain" = "aichat --role explain";
    "ai-review" = "aichat --role reviewer";
  };
}
