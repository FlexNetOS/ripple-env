# Library utility functions for ros2-humble-env
# Inspired by RGBCube/ncc and GustavoWidman/nix
{
  lib,
  inputs,
  ...
}:
let
  inherit (builtins) readDir attrNames filter;
in
{
  # Import all nix files from a directory recursively
  collectNixFiles =
    dir:
    let
      entries = readDir dir;
      nixFiles = filter (n: entries.${n} == "regular" && lib.hasSuffix ".nix" n && n != "default.nix") (
        attrNames entries
      );
      directories = filter (n: entries.${n} == "directory") (attrNames entries);
    in
    (map (f: dir + "/${f}") nixFiles)
    ++ (lib.concatMap (d: lib.collectNixFiles (dir + "/${d}")) directories);

  # Check if system is Darwin (macOS)
  isDarwin = system: lib.hasSuffix "-darwin" system;

  # Check if system is Linux
  isLinux = system: lib.hasSuffix "-linux" system;

  # Get architecture from system string
  getArch =
    system:
    let
      parts = lib.splitString "-" system;
    in
    lib.head parts;

  # Helper to create a simple enable option
  mkEnableOpt =
    description:
    lib.mkOption {
      type = lib.types.bool;
      default = false;
      inherit description;
    };

  # Merge multiple attribute sets deeply
  deepMerge = lib.foldl' lib.recursiveUpdate { };
}
