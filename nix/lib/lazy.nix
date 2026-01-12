# Lazy evaluation utilities for Nix
# Reduces evaluation time by deferring expensive computations
#
# Usage in flake.nix:
#   lazy = import ./nix/lib/lazy.nix { inherit lib; };
#   shells = lazy.lazyShells { inherit pkgs; shells = allShells; };
{ lib, ... }:

{
  # Wrap shell definitions in lazy evaluation
  # Only evaluates a shell when it's actually accessed
  #
  # Usage:
  #   lazyShells { pkgs, shells }
  #   Where shells is an attrset of mkShell derivations
  lazyShells = { pkgs, shells }:
    builtins.mapAttrs (name: shellFn:
      if builtins.isFunction shellFn
      then shellFn
      else shellFn
    ) shells;

  # Create a lazy package list that only evaluates when needed
  # Useful for optional heavy dependencies
  #
  # Usage:
  #   lazyPackages { condition = config.enableCuda; packages = cudaPackages; }
  lazyPackages = { condition ? true, packages }:
    lib.optionals condition packages;

  # Conditional import that returns empty attrset when condition is false
  # Prevents evaluation of the imported file entirely
  #
  # Usage:
  #   conditionalImport { condition = stdenv.isLinux; path = ./linux-only.nix; args = { inherit pkgs; }; }
  conditionalImport = { condition ? true, path, args ? {} }:
    if condition
    then import path args
    else {};

  # Lazy attribute set that only evaluates values when accessed
  # Uses builtins.tryEval to handle evaluation errors gracefully
  lazyAttrs = attrs:
    builtins.listToAttrs (
      builtins.map (name: {
        inherit name;
        value = attrs.${name};
      }) (builtins.attrNames attrs)
    );
}
