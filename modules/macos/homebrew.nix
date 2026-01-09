# Homebrew integration for macOS
# Manages packages that work better through Homebrew
# Based on GustavoWidman/nix patterns
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
  # Detect architecture: Apple Silicon uses /opt/homebrew, Intel uses /usr/local
  isAarch64 = pkgs.stdenv.hostPlatform.isAarch64;
  homebrewPrefix = if isAarch64 then "/opt/homebrew" else "/usr/local";
in
{
  # Note: This module provides configuration for nix-darwin with nix-homebrew
  # For standalone home-manager, Homebrew must be managed separately

  # Homebrew environment setup (auto-detects Apple Silicon vs Intel)
  home.sessionVariables = {
    HOMEBREW_PREFIX = homebrewPrefix;
    HOMEBREW_CELLAR = "${homebrewPrefix}/Cellar";
    HOMEBREW_REPOSITORY = homebrewPrefix;
  };

  # Add Homebrew to PATH
  home.sessionPath = [
    "${homebrewPrefix}/bin"
    "${homebrewPrefix}/sbin"
  ];

  # Homebrew aliases
  home.shellAliases = {
    "brewup" = "brew update && brew upgrade && brew cleanup";
    "brewclean" = "brew cleanup --prune=all";
  };

  # For nix-darwin systems, the following would be added to darwin configuration:
  # homebrew = {
  #   enable = true;
  #   onActivation = {
  #     autoUpdate = true;
  #     cleanup = "zap";
  #     upgrade = true;
  #   };
  #
  #   # Taps
  #   taps = [
  #     "homebrew/cask"
  #     "homebrew/cask-fonts"
  #   ];
  #
  #   # Formulae (CLI tools)
  #   brews = [
  #     # Build dependencies sometimes needed
  #     "openssl@3"
  #     "libyaml"
  #   ];
  #
  #   # Casks (GUI applications)
  #   casks = [
  #     # Terminal emulators
  #     "ghostty"
  #     "kitty"
  #
  #     # Development tools
  #     "visual-studio-code"
  #     "docker"
  #
  #     # Utilities
  #     "rectangle"  # Window management
  #     "stats"      # System monitoring
  #   ];
  # };

  # Create a Brewfile template for manual setup
  home.file.".config/homebrew/Brewfile" = {
    text = ''
      # Homebrew Bundle file for ROS2 development on macOS
      # Run with: brew bundle --file=~/.config/homebrew/Brewfile

      # Taps
      tap "homebrew/cask"
      tap "homebrew/cask-fonts"

      # Build dependencies
      brew "openssl@3"
      brew "libyaml"
      brew "cmake"
      brew "pkg-config"

      # Fonts
      cask "font-fira-code-nerd-font"
      cask "font-jetbrains-mono-nerd-font"
      cask "font-hack-nerd-font"

      # Terminal
      cask "ghostty"

      # Development
      cask "docker"

      # Utilities
      cask "rectangle"  # Window management
      cask "stats"      # System monitoring
    '';
  };
}
