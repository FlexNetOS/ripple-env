# NixOS ISO Installer Image Configuration
# Builds a bootable ISO for bare metal installation
#
# Build command (development):
#   nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage
#
# Build command (production/stable):
#   nix build .#nixosConfigurations.iso-ros2-stable.config.system.build.isoImage
#
# Write to USB:
#   dd if=result/iso/nixos-*.iso of=/dev/sdX bs=4M status=progress
#
# Supply Chain Security:
#   - Use *-stable variants for production deployments
#   - Stable uses nixos-24.11 with vetted packages
{ inputs, pkgs, lib, isStable ? false, ... }:

let
  nixpkgsInput = if isStable then inputs.nixpkgs-stable else inputs.nixpkgs;
  channelLabel = if isStable then "stable" else "unstable";
in
nixpkgsInput.lib.nixosSystem {
  system = "x86_64-linux";

  modules = [
    # ISO installer base
    "${nixpkgsInput}/nixos/modules/installer/cd-dvd/installation-cd-minimal.nix"

    # Security hardening module
    ./security-hardening.nix

    # ROS2 development environment
    ({ config, pkgs, lib, ... }: {
      # Enable security hardening (standard level for installer)
      security.hardening = {
        enable = true;
        level = "standard";
        auditd.enable = false;  # Not needed for installer
      };
      # ISO-specific configuration
      isoImage = {
        isoName = lib.mkForce "nixos-ros2-${config.system.nixos.label}-x86_64-linux.iso";
        volumeID = "NIXOS_ROS2";
        makeEfiBootable = true;
        makeUsbBootable = true;
      };

      # Use new image module format (fixes deprecation warning)
      image = {
        fileName = "nixos-ros2-${config.system.nixos.label}-x86_64-linux.iso";
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
