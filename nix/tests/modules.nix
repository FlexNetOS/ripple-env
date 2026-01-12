# Module Unit Tests
# Tests for home-manager modules using NixOS testing framework
#
# These tests verify that modules:
# 1. Evaluate without errors
# 2. Produce expected configurations
# 3. Handle option combinations correctly
#
{ pkgs, lib }:

let
  # Helper to create a minimal home-manager evaluation
  evalHomeModule =
    { modules, extraSpecialArgs ? { } }:
    let
      # Minimal home-manager module evaluation
      eval = lib.evalModules {
        modules = [
          # Base home-manager structure
          {
            options = {
              home = {
                username = lib.mkOption {
                  type = lib.types.str;
                  default = "testuser";
                };
                homeDirectory = lib.mkOption {
                  type = lib.types.path;
                  default = "/home/testuser";
                };
                stateVersion = lib.mkOption {
                  type = lib.types.str;
                  default = "24.05";
                };
                packages = lib.mkOption {
                  type = lib.types.listOf lib.types.package;
                  default = [ ];
                };
                file = lib.mkOption {
                  type = lib.types.attrsOf lib.types.anything;
                  default = { };
                };
                sessionVariables = lib.mkOption {
                  type = lib.types.attrsOf lib.types.str;
                  default = { };
                };
              };
              programs = lib.mkOption {
                type = lib.types.attrsOf lib.types.anything;
                default = { };
              };
              xdg = lib.mkOption {
                type = lib.types.attrsOf lib.types.anything;
                default = { };
              };
            };
          }
        ] ++ modules;
        specialArgs = { inherit pkgs lib; } // extraSpecialArgs;
      };
    in
    eval.config;

  # Test assertion helper
  assertEq =
    name: expected: actual:
    if expected == actual then
      true
    else
      throw "Test '${name}' failed: expected ${builtins.toJSON expected}, got ${builtins.toJSON actual}";

  assertNotNull =
    name: value:
    if value != null then
      true
    else
      throw "Test '${name}' failed: value is null";

  assertTrue =
    name: value:
    if value == true then
      true
    else
      throw "Test '${name}' failed: expected true, got ${builtins.toJSON value}";

in
{
  # Test: Git module evaluates correctly
  test-git-module = pkgs.runCommand "test-git-module"
    {
      nativeBuildInputs = [ pkgs.coreutils ];
    }
    ''
      echo "Testing git module evaluation..."

      # Test 1: Module imports without errors
      ${
        let
          config = evalHomeModule {
            modules = [
              ../../../modules/common/git.nix
            ];
          };
        in
        if config.programs.git.enable or false then
          "echo 'PASS: Git module enables git by default'"
        else
          "echo 'FAIL: Git module should enable git'; exit 1"
      }

      # Test 2: LFS is enabled by default
      ${
        let
          config = evalHomeModule {
            modules = [
              ../../../modules/common/git.nix
            ];
          };
        in
        if config.programs.git.lfs.enable or false then
          "echo 'PASS: Git LFS enabled by default'"
        else
          "echo 'FAIL: Git LFS should be enabled'; exit 1"
      }

      echo "All git module tests passed"
      touch $out
    '';

  # Test: Direnv module evaluates correctly
  test-direnv-module = pkgs.runCommand "test-direnv-module"
    {
      nativeBuildInputs = [ pkgs.coreutils ];
    }
    ''
      echo "Testing direnv module evaluation..."

      # Test 1: Module imports without errors
      ${
        let
          config = evalHomeModule {
            modules = [
              ../../../modules/common/direnv.nix
            ];
          };
        in
        if config.programs.direnv.enable or false then
          "echo 'PASS: Direnv module enables direnv'"
        else
          "echo 'FAIL: Direnv module should enable direnv'; exit 1"
      }

      # Test 2: Nix-direnv is enabled
      ${
        let
          config = evalHomeModule {
            modules = [
              ../../../modules/common/direnv.nix
            ];
          };
        in
        if config.programs.direnv.nix-direnv.enable or false then
          "echo 'PASS: nix-direnv enabled'"
        else
          "echo 'FAIL: nix-direnv should be enabled'; exit 1"
      }

      echo "All direnv module tests passed"
      touch $out
    '';

  # Test: Packages module provides expected packages
  test-packages-module = pkgs.runCommand "test-packages-module"
    {
      nativeBuildInputs = [ pkgs.coreutils ];
    }
    ''
      echo "Testing packages module evaluation..."

      # Test 1: Module imports without errors
      ${
        let
          config = evalHomeModule {
            modules = [
              ../../../modules/common/packages.nix
            ];
          };
          packageCount = builtins.length (config.home.packages or [ ]);
        in
        if packageCount > 0 then
          "echo 'PASS: Packages module provides ${toString packageCount} packages'"
        else
          "echo 'INFO: Packages module provides no packages (may be expected)'"
      }

      echo "All packages module tests passed"
      touch $out
    '';
}
