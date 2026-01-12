# Nix Test Suite for ripple-env
# Unit tests for Nix modules using testers.runNixOSTest
#
# Run tests:
#   nix build .#checks.x86_64-linux.module-tests
#   nix flake check
#
{ pkgs ? import <nixpkgs> { }, lib ? pkgs.lib }:

let
  # Import test modules
  moduleTests = import ./modules.nix { inherit pkgs lib; };
  libTests = import ./lib.nix { inherit pkgs lib; };
  shellTests = import ./shells.nix { inherit pkgs lib; };

in
{
  # Module unit tests
  inherit (moduleTests)
    test-git-module
    test-direnv-module
    test-packages-module;

  # Library function tests
  inherit (libTests)
    test-lib-functions;

  # Shell configuration tests
  inherit (shellTests)
    test-shell-packages;
}
