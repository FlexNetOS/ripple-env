# Shell Configuration Tests
# Tests for devShell configurations
#
{ pkgs, lib }:

let
  # Import modular packages to test
  modularPackages = import ../packages { inherit pkgs lib; };

in
{
  # Test: Shell packages are valid
  test-shell-packages = pkgs.runCommand "test-shell-packages"
    {
      nativeBuildInputs = [ pkgs.coreutils ];
    }
    ''
      echo "Testing shell package configurations..."

      # Test 1: Default shell packages list exists and is non-empty
      ${
        let
          defaultPkgs = modularPackages.defaultShell or [ ];
          count = builtins.length defaultPkgs;
        in
        if count > 0 then
          "echo 'PASS: Default shell has ${toString count} packages'"
        else
          "echo 'FAIL: Default shell should have packages'; exit 1"
      }

      # Test 2: Base packages exist
      ${
        let
          basePkgs = modularPackages.base or [ ];
          count = builtins.length basePkgs;
        in
        if count > 0 then
          "echo 'PASS: Base packages has ${toString count} packages'"
        else
          "echo 'FAIL: Base packages should exist'; exit 1"
      }

      # Test 3: Dev tools packages exist
      ${
        let
          devPkgs = modularPackages.devTools or [ ];
          count = builtins.length devPkgs;
        in
        if count > 0 then
          "echo 'PASS: Dev tools has ${toString count} packages'"
        else
          "echo 'FAIL: Dev tools should exist'; exit 1"
      }

      # Test 4: All default packages are valid derivations
      ${
        let
          defaultPkgs = modularPackages.defaultShell or [ ];
          allValid = builtins.all (p: p ? type && p.type == "derivation") defaultPkgs;
        in
        if allValid then
          "echo 'PASS: All default shell packages are valid derivations'"
        else
          "echo 'FAIL: Some packages are not valid derivations'; exit 1"
      }

      echo "All shell package tests passed"
      touch $out
    '';
}
