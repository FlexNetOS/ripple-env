# Minimal packages for fastest possible shell startup
# Use this for CI, scripting, and quick operations
#
# Shell startup time target: < 1 second
# Evaluation time target: < 5 seconds
{ pkgs, ... }:

with pkgs; [
  # Absolute essentials only
  git
  pixi

  # Required for colcon builds
  python313
  python313Packages.pip

  # Nix development
  nixfmt-rfc-style
]
