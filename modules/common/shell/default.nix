# Shell configuration
# Multi-shell setup with nushell, zsh, and bash
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
  imports = [
    ./nushell.nix
    ./zsh.nix
    ./bash.nix
    ./zoxide.nix
    ./starship.nix
  ];

  # Common shell utilities
  home.packages = with pkgs; [
    # Shell enhancements
    carapace  # Completion engine
    vivid     # LS_COLORS generator
  ];
}
