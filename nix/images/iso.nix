# NixOS ISO Installer Image Configuration
# Builds a bootable ISO for bare metal installation
#
# Build command:
#   nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage
#
# Write to USB:
#   dd if=result/iso/nixos-*.iso of=/dev/sdX bs=4M status=progress
{ inputs, pkgs, lib, ... }:

inputs.nixpkgs.lib.nixosSystem {
  system = "x86_64-linux";

  modules = [
    # ISO installer base
    "${inputs.nixpkgs}/nixos/modules/installer/cd-dvd/installation-cd-minimal.nix"

    # ROS2 development environment
    ({ config, pkgs, lib, ... }: {
      # ISO-specific configuration
      isoImage = {
        isoName = "nixos-ros2-${config.system.nixos.label}-x86_64-linux.iso";
        volumeID = "NIXOS_ROS2";
        makeEfiBootable = true;
        makeUsbBootable = true;
      };

      # Nix configuration
      nix.settings = {
        experimental-features = [ "nix-command" "flakes" ];
        auto-optimise-store = true;
      };

      # Minimal packages for installation
      environment.systemPackages = with pkgs; [
        git
        curl
        wget
        helix
        neovim
        nix-output-monitor
        nix-tree
      ];

      # System state version
      system.stateVersion = "24.05";
    })
  ];
}
