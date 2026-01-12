# Library Function Tests
# Tests for utility functions in lib/
#
{ pkgs, lib }:

let
  # Import the library functions
  rippleLib = import ../../lib { inherit lib; inputs = { }; };

  # Helper for assertions
  assert' =
    name: condition:
    if condition then
      "echo 'PASS: ${name}'"
    else
      "echo 'FAIL: ${name}'; exit 1";

in
{
  # Test: Library functions exist and work correctly
  test-lib-functions = pkgs.runCommand "test-lib-functions"
    {
      nativeBuildInputs = [ pkgs.coreutils ];
    }
    ''
      echo "Testing library functions..."

      # Test 1: collectNixFiles function exists
      ${
        if builtins.hasAttr "collectNixFiles" rippleLib then
          "echo 'PASS: collectNixFiles function exists'"
        else
          "echo 'FAIL: collectNixFiles function missing'; exit 1"
      }

      # Test 2: isDarwin function exists
      ${
        if builtins.hasAttr "isDarwin" rippleLib then
          "echo 'PASS: isDarwin function exists'"
        else
          "echo 'FAIL: isDarwin function missing'; exit 1"
      }

      # Test 3: isLinux function exists
      ${
        if builtins.hasAttr "isLinux" rippleLib then
          "echo 'PASS: isLinux function exists'"
        else
          "echo 'FAIL: isLinux function missing'; exit 1"
      }

      # Test 4: mkEnableOpt function exists
      ${
        if builtins.hasAttr "mkEnableOpt" rippleLib then
          "echo 'PASS: mkEnableOpt function exists'"
        else
          "echo 'FAIL: mkEnableOpt function missing'; exit 1"
      }

      # Test 5: deepMerge function exists
      ${
        if builtins.hasAttr "deepMerge" rippleLib then
          "echo 'PASS: deepMerge function exists'"
        else
          "echo 'FAIL: deepMerge function missing'; exit 1"
      }

      # Test 5b: getArch function exists
      ${
        if builtins.hasAttr "getArch" rippleLib then
          "echo 'PASS: getArch function exists'"
        else
          "echo 'FAIL: getArch function missing'; exit 1"
      }

      # Test 6: Platform detection returns boolean
      ${
        let
          # isDarwin and isLinux take system string, not stdenv
          isDarwinResult = rippleLib.isDarwin "x86_64-darwin";
          isLinuxResult = rippleLib.isLinux "x86_64-linux";
        in
        if builtins.isBool isDarwinResult && builtins.isBool isLinuxResult then
          "echo 'PASS: Platform detection returns booleans'"
        else
          "echo 'FAIL: Platform detection should return booleans'; exit 1"
      }

      # Test 7: Platform detection is accurate
      ${
        let
          isDarwinForDarwin = rippleLib.isDarwin "aarch64-darwin";
          isDarwinForLinux = rippleLib.isDarwin "x86_64-linux";
          isLinuxForLinux = rippleLib.isLinux "x86_64-linux";
          isLinuxForDarwin = rippleLib.isLinux "aarch64-darwin";
        in
        if isDarwinForDarwin && !isDarwinForLinux && isLinuxForLinux && !isLinuxForDarwin then
          "echo 'PASS: Platform detection is accurate'"
        else
          "echo 'FAIL: Platform detection gave wrong results'; exit 1"
      }

      # Test 8: deepMerge works correctly (takes a list)
      ${
        let
          merged = rippleLib.deepMerge [ { a = 1; b = { c = 2; }; } { b = { d = 3; }; } ];
        in
        if merged.a == 1 && merged.b.c == 2 && merged.b.d == 3 then
          "echo 'PASS: deepMerge correctly merges nested attrs'"
        else
          "echo 'FAIL: deepMerge did not merge correctly'; exit 1"
      }

      # Test 9: getArch extracts architecture
      ${
        let
          arch = rippleLib.getArch "x86_64-linux";
        in
        if arch == "x86_64" then
          "echo 'PASS: getArch correctly extracts architecture'"
        else
          "echo 'FAIL: getArch returned ${arch} instead of x86_64'; exit 1"
      }

      echo "All library function tests passed"
      touch $out
    '';
}
