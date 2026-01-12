# NixOS image builders aggregator
# Usage in flake.nix:
#   images = import ./nix/images { inherit inputs pkgs lib; };
#   nixosConfigurations.wsl-ripple = images.wsl;
{ inputs, pkgs, lib, ... }:

{
  # WSL2 image configuration
  wsl = import ./wsl.nix { inherit inputs pkgs lib; };

  # ISO installer image configuration
  iso = import ./iso.nix { inherit inputs pkgs lib; };

  # QEMU/VirtualBox VM image configuration
  vm = import ./vm.nix { inherit inputs pkgs lib; };
}
