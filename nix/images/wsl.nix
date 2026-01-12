# WSL2 NixOS Image Configuration
# Builds a tarball that can be imported into Windows Subsystem for Linux
#
# Build command:
#   nix build .#nixosConfigurations.wsl-ros2.config.system.build.tarballBuilder
#
# Import command (PowerShell):
#   wsl --import NixOS-ROS2 $env:USERPROFILE\WSL\NixOS-ROS2 result/nixos-wsl.tar.gz
#
# Launch:
#   wsl -d NixOS-ROS2
{ inputs, pkgs, lib, ... }:

inputs.nixos-wsl.lib.nixosSystem {
  system = "x86_64-linux";

  modules = [
    # NixOS-WSL base module
    inputs.nixos-wsl.nixosModules.wsl

    # Security hardening module
    ./security-hardening.nix

    # ROS2 Humble development environment configuration
    ({ config, pkgs, lib, ... }: {
      # Enable security hardening (minimal level for dev)
      security.hardening = {
        enable = true;
        level = "minimal";  # Use minimal for development convenience
        auditd.enable = false;  # Disable audit in WSL
      };
      # WSL-specific configuration
      wsl = {
        enable = true;
        defaultUser = "nixos";
        nativeSystemd = true;

        # Enable Windows interop
        interop = {
          register = true;
          includePath = true;
        };

        # WSL-specific settings
        wslConf = {
          automount = {
            enabled = true;
            root = "/mnt";
            options = "metadata,umask=22,fmask=11";
          };
          network = {
            hostname = "nixos-ros2";
            generateHosts = true;
            generateResolvConf = true;
          };
        };
      };

      # System configuration
      networking.hostName = "nixos-ros2";
      time.timeZone = "UTC";

      # Nix configuration
      nix = {
        settings = {
          experimental-features = [ "nix-command" "flakes" ];
          trusted-users = [ "@wheel" "nixos" ];
          auto-optimise-store = true;
        };
        gc = {
          automatic = true;
          dates = "weekly";
          options = "--delete-older-than 7d";
        };
      };

      # User configuration
      users.users.nixos = {
        isNormalUser = true;
        extraGroups = [ "wheel" "docker" "video" "audio" ];
        shell = pkgs.zsh;
      };

      # Enable sudo without password for wheel group (dev convenience)
      security.sudo.wheelNeedsPassword = false;

      # System packages available to all users
      environment.systemPackages = with pkgs; [
        # Core utilities
        git
        gh
        curl
        wget
        jq
        yq
        ripgrep
        fd
        bat
        eza
        htop
        btop

        # Editors
        helix
        neovim

        # Nix tools
        nix-output-monitor
        nix-tree
        nixfmt-rfc-style
        nil

        # Development
        direnv
        nix-direnv
        zoxide
        starship

        # Shell
        zsh
        nushell

        # Container tools
        docker
        docker-compose

        # Python
        python313
        python313Packages.pip
        python313Packages.virtualenv

        # Build tools
        gcc
        gnumake
        cmake
        pkg-config

        # Version control
        jujutsu
        lazygit
      ];

      # Enable Docker
      virtualisation.docker = {
        enable = true;
        enableOnBoot = true;
      };

      # Enable Zsh system-wide
      programs.zsh.enable = true;

      # Direnv integration
      programs.direnv = {
        enable = true;
        nix-direnv.enable = true;
      };

      # Git configuration
      programs.git = {
        enable = true;
        lfs.enable = true;
      };

      # SSH server (for remote development)
      services.openssh = {
        enable = true;
        settings = {
          PermitRootLogin = "no";
          PasswordAuthentication = false;
        };
      };

      # Enable systemd services
      systemd.services = {
        # Ensure Docker starts properly in WSL
        docker.wantedBy = [ "multi-user.target" ];
      };

      # Environment variables
      environment.variables = {
        EDITOR = "hx";
        VISUAL = "hx";
      };

      # Shell aliases
      environment.shellAliases = {
        ll = "eza -la";
        la = "eza -a";
        l = "eza -l";
        cat = "bat";
        vim = "hx";
        vi = "hx";
      };

      # System state version (NixOS release version)
      system.stateVersion = "24.05";
    })
  ];
}
