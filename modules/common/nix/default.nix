# Nix configuration settings
# Enhanced nix settings for optimal development experience
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
  # Nix settings (for NixOS/nix-darwin systems)
  nix.settings = {
    # Enable experimental features
    experimental-features = mkDefault [
      "nix-command"
      "flakes"
    ];

    # Optimize store automatically
    auto-optimise-store = mkDefault true;

    # Build settings
    max-jobs = mkDefault "auto";
    cores = mkDefault 0; # Use all available cores

    # Keep derivations and outputs for development
    keep-derivations = mkDefault true;
    keep-outputs = mkDefault true;

    # Substituters (binary caches)
    substituters = mkDefault [
      "https://cache.nixos.org"
      "https://nix-community.cachix.org"
      "https://cache.garnix.io"
    ];

    trusted-public-keys = mkDefault [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "cache.garnix.io:CTFPyKSLcx5RMJKfLo5EEPUObbA78b0YQ2DTCJXqr9g="
    ];

    # Trust users for development
    trusted-users = mkDefault [
      "root"
      "@wheel"
      "@admin"
    ];
  };

  # Garbage collection settings
  nix.gc = {
    automatic = mkDefault true;
    options = mkDefault "--delete-older-than 7d";
  };
}
