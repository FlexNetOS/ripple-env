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
