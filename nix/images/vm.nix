# NixOS VM Image Configuration
# Builds a QEMU/VirtualBox compatible VM image
#
# Build command:
#   nix build .#nixosConfigurations.vm-ros2.config.system.build.vm
#
# Run the VM:
#   ./result/bin/run-nixos-ros2-vm
{ inputs, pkgs, lib, ... }:

inputs.nixpkgs.lib.nixosSystem {
  system = "x86_64-linux";

  modules = [
    # QEMU VM base module
    "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"

    # ROS2 development environment
    ({ config, pkgs, lib, ... }: {
      # VM-specific configuration
      virtualisation = {
        memorySize = 4096;  # 4GB RAM
        cores = 4;
        diskSize = 20480;   # 20GB disk
        graphics = false;    # Headless by default

        # Forward SSH port
        forwardPorts = [
          { from = "host"; host.port = 2222; guest.port = 22; }
        ];

        # Shared folder with host
        sharedDirectories = {
          ros2-workspace = {
            source = "/tmp/ros2-workspace";
            target = "/home/nixos/ros2-workspace";
          };
        };
      };

      # Nix configuration
      nix.settings = {
        experimental-features = [ "nix-command" "flakes" ];
        auto-optimise-store = true;
      };

      # User configuration
      users.users.nixos = {
        isNormalUser = true;
        extraGroups = [ "wheel" "docker" ];
        password = "nixos";  # Default password for VM
        shell = pkgs.zsh;
      };

      # Enable sudo without password
      security.sudo.wheelNeedsPassword = false;

      # System packages
      environment.systemPackages = with pkgs; [
        git
        gh
        curl
        wget
        jq
        helix
        neovim
        direnv
        nix-direnv
        nix-output-monitor
        nix-tree
        nixfmt-rfc-style
        python313
        docker
        docker-compose
      ];

      # Enable Docker
      virtualisation.docker.enable = true;

      # Enable SSH
      services.openssh = {
        enable = true;
        settings.PermitRootLogin = "no";
      };

      # Enable Zsh
      programs.zsh.enable = true;

      # Environment
      environment.variables = {
        EDITOR = "hx";
        VISUAL = "hx";
      };

      # System state version
      system.stateVersion = "24.05";
    })
  ];
}
