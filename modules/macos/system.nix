# macOS system configuration
# System-level settings for nix-darwin
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
  # Note: These settings are for nix-darwin
  # For standalone home-manager, most of these are not applicable

  # macOS defaults that improve development experience
  # Apply these manually with: defaults write <domain> <key> <value>

  home.file.".config/macos/apply-defaults.sh" = {
    executable = true;
    text = ''
      #!/usr/bin/env bash
      # macOS defaults for development
      # Run this script to apply recommended settings

      echo "Applying macOS defaults..."

      # Finder: Show hidden files
      defaults write com.apple.finder AppleShowAllFiles -bool true

      # Finder: Show all file extensions
      defaults write NSGlobalDomain AppleShowAllExtensions -bool true

      # Finder: Show path bar
      defaults write com.apple.finder ShowPathbar -bool true

      # Finder: Show status bar
      defaults write com.apple.finder ShowStatusBar -bool true

      # Finder: Allow text selection in Quick Look
      defaults write com.apple.finder QLEnableTextSelection -bool true

      # Finder: Disable warning when changing file extension
      defaults write com.apple.finder FXEnableExtensionChangeWarning -bool false

      # Dock: Minimize windows into application icon
      defaults write com.apple.dock minimize-to-application -bool true

      # Dock: Speed up animations
      defaults write com.apple.dock autohide-delay -float 0
      defaults write com.apple.dock autohide-time-modifier -float 0.5

      # Dock: Don't show recent applications
      defaults write com.apple.dock show-recents -bool false

      # Keyboard: Fast key repeat
      defaults write NSGlobalDomain KeyRepeat -int 2
      defaults write NSGlobalDomain InitialKeyRepeat -int 15

      # Keyboard: Disable smart quotes and dashes (annoying for coding)
      defaults write NSGlobalDomain NSAutomaticQuoteSubstitutionEnabled -bool false
      defaults write NSGlobalDomain NSAutomaticDashSubstitutionEnabled -bool false

      # Trackpad: Enable tap to click
      defaults write com.apple.driver.AppleBluetoothMultitouch.trackpad Clicking -bool true

      # Terminal: Enable Secure Keyboard Entry
      defaults write com.apple.terminal SecureKeyboardEntry -bool true

      # TextEdit: Use plain text mode by default
      defaults write com.apple.TextEdit RichText -int 0

      # Screenshots: Save to custom location
      mkdir -p ~/Pictures/Screenshots
      defaults write com.apple.screencapture location -string "~/Pictures/Screenshots"

      # Screenshots: Save as PNG
      defaults write com.apple.screencapture type -string "png"

      # Disable "Are you sure you want to open this application?"
      defaults write com.apple.LaunchServices LSQuarantine -bool false

      # Disable auto-correct
      defaults write NSGlobalDomain NSAutomaticSpellingCorrectionEnabled -bool false

      echo "Defaults applied. Some changes require logout/restart."
      echo "Restarting affected applications..."

      killall Finder 2>/dev/null || true
      killall Dock 2>/dev/null || true
      killall SystemUIServer 2>/dev/null || true

      echo "Done!"
    '';
  };
}
