# Nix Test Suite for ripple-env
#
# Provides:
# - Unit tests for Nix modules
# - Library tests
# - Shell configuration tests
# - Optional NixOS VM image tests (only when `inputs` is provided)
#
# Usage:
#   nix flake check
#   nix build .#checks.x86_64-linux.test-git-module
#
{ inputs ? null
, pkgs ? import <nixpkgs> { }
, lib ? pkgs.lib
, system ? (pkgs.stdenv.hostPlatform.system or "x86_64-linux")
, ...
}:

let
  # Unit tests
  moduleTests = import ./modules.nix { inherit pkgs lib; };
  libTests = import ./lib.nix { inherit pkgs lib; };
  shellTests = import ./shells.nix { inherit pkgs lib; };

  # Image tests (NixOS VM tests) are only available when the flake `inputs`
  # are provided (so we can reference nixos modules deterministically).
  imageTestsDef = if inputs == null then null else import ./image-tests.nix { inherit inputs pkgs lib; };
  imageTestDrvs = if imageTestsDef == null then { } else imageTestsDef.mkTests { inherit system; };

  imageChecks = if imageTestsDef == null then { } else {
    inherit (imageTestDrvs)
      basic-services
      docker-services
      dev-tools
      security-hardening
      network;

    all-image-tests = pkgs.runCommand "all-image-tests" { } ''
      echo "All image tests passed"
      mkdir -p $out
      touch $out/success
    '';
  };

in
{
  inherit (moduleTests)
    test-git-module
    test-direnv-module
    test-packages-module;

  inherit (libTests)
    test-lib-functions;

  inherit (shellTests)
    test-shell-packages;
}
// imageChecks
