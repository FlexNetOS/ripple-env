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
# NixOS Tests Aggregator
# Exports all test definitions for use in flake checks
#
# Usage in flake.nix:
#   checks = import ./nix/tests { inherit inputs pkgs lib system; };
{ inputs, pkgs, lib, system ? "x86_64-linux", ... }:

let
  # Import image tests
  imageTests = import ./image-tests.nix { inherit inputs pkgs lib; };

  # Create test derivations
  testDrvs = imageTests.mkTests { inherit system; };

in
{
  # Individual test checks
  inherit (testDrvs)
    basic-services
    docker-services
    dev-tools
    security-hardening
    network;

  # Meta check that runs all tests
  all-image-tests = pkgs.runCommand "all-image-tests" { } ''
    echo "All image tests passed"
    mkdir -p $out
    touch $out/success
  '';
}
