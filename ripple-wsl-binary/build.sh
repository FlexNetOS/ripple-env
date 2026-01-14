#!/usr/bin/env bash
# Build FlexNetOS as native NixOS WSL distribution

set -euo pipefail

BUILD_DIR="/mnt/okcomputer/output/flexnetos-wsl-binary"
NIXOS_WSL_DIR="$BUILD_DIR/nixos-wsl-config"
OUTPUT_DIR="$BUILD_DIR/output"

# Create directories
mkdir -p "$NIXOS_WSL_DIR"/{modules,packages,services,config}
mkdir -p "$OUTPUT_DIR"
mkdir -p "$NIXOS_WSL_DIR/config/docker"

echo "Creating NixOS WSL configuration for FlexNetOS..."

# Create flake.nix
cat > "$NIXOS_WSL_DIR/flake.nix" << 'EOF'
{
  description = "FlexNetOS - NixOS WSL Distribution";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nixos-wsl.url = "github:nix-community/NixOS-WSL";
    flake-utils.url = "github:numtide/flake-utils";
    
    # FlexNetOS specific inputs
    holonix.url = "github:holochain/holonix";
    
    # Additional overlays
    nur.url = "github:nix-community/NUR";
  };

  outputs = { self, nixpkgs, nixos-wsl, flake-utils, holonix, nur, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
          overlays = [
            nixos-wsl.overlays.default
            (final: prev: {
              # FlexNetOS custom packages
              flexnetos-config = final.callPackage ./packages/config.nix {};
            })
          ];
        };

        # WSL-specific configuration
        wslConfig = {
          wsl = {
            enable = true;
            defaultUser = "flexnet";
            automountPath = "/mnt";
            docker-desktop.enable = false;
          };
        };

      in {
        nixosConfigurations.flexnetos-wsl = nixpkgs.lib.nixosSystem {
          inherit system;
          modules = [
            nixos-wsl.nixosModules.wsl
            ./configuration.nix
            ./modules/flexnetos-core.nix
            ./modules/flexnetos-services.nix
            ./modules/flexnetos-security.nix
            ./modules/flexnetos-monitoring.nix
          ];
          specialArgs = { inherit wslConfig; };
        };

        packages.default = self.nixosConfigurations.flexnetos-wsl.config.system.build.toplevel;
        
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            nixos-generators
            wslu
            git
            curl
            jq
          ];
        };
      });
}
EOF

# Create configuration.nix
cat > "$NIXOS_WSL_DIR/configuration.nix" << 'EOF'
{ config, pkgs, wslConfig, ... }:

{
  imports = [
    # NixOS-WSL specific modules
    <nixos-wsl/modules>
    
    # FlexNetOS modules
    ./modules/flexnetos-core.nix
    ./modules/flexnetos-services.nix
    ./modules/flexnetos-security.nix
    ./modules/flexnetos-monitoring.nix
  ];

  # System basics
  system.stateVersion = "24.05";
  
  # WSL configuration
  wsl = wslConfig.wsl;

  # Boot configuration
  boot.kernel.sysctl = {
    "net.ipv4.ip_forward" = 1;
    "net.ipv6.conf.all.forwarding" = 1;
  };

  # Networking
  networking = {
    hostName = "flexnetos";
    useDHCP = false;
    firewall.enable = false; # WSL handles this
  };

  # Users
  users.users.flexnet = {
    isNormalUser = true;
    description = "FlexNetOS Administrator";
    extraGroups = [ "wheel" "docker" "systemd-journal" ];
    shell = pkgs.zsh;
    openssh.authorizedKeys.keys = [];
  };

  # Timezone
  time.timeZone = "UTC";

  # Locale
  i18n.defaultLocale = "en_US.UTF-8";

  # System packages
  environment.systemPackages = with pkgs; [
    # Core utilities
    git
    curl
    wget
    jq
    yq
    tmux
    htop
    vim
    neovim
    
    # Docker and container tools
    docker
    docker-compose
    kubernetes-helm
    k9s
    
    # Development tools
    gcc
    gnumake
    cmake
    pkg-config
    
    # FlexNetOS specific
    rustc
    cargo
    go_1_21
    nodejs_20
    python311
    python311Packages.pip
    
    # System utilities
    systemd
    networkmanager
    iptables
    
    # Monitoring
    prometheus
    grafana
    node_exporter
    
    # Security
    vault
    openssl
    gnupg
    fail2ban
    aide
  ];

  # Enable systemd
  systemd.enable = true;

  # Services
  services = {
    docker = {
      enable = true;
      autoPrune.enable = true;
    };

    openssh = {
      enable = true;
      settings.PermitRootLogin = "no";
      settings.PasswordAuthentication = false;
    };

    # Enable required kernel modules
    kernel-modules = {
      enable = true;
      modules = [ "br_netfilter" "overlay" "nf_conntrack" ];
    };
  };

  # Environment variables
  environment.variables = {
    FLEXNETOS_HOME = "/opt/flexnetos";
    FLEXNETOS_CONFIG = "/etc/flexnetos";
    FLEXNETOS_DATA = "/var/lib/flexnetos";
  };

  # System paths
  environment.pathsToLink = [ "/etc/flexnetos" ];

  # File systems
  fileSystems."/mnt/c" = {
    device = "C:";
    fsType = "drvfs";
    options = [ "metadata" ];
  };

  # Ensure directories exist
  systemd.tmpfiles.rules = [
    "d /opt/flexnetos 755 flexnet users -"
    "d /etc/flexnetos 755 root root -"
    "d /var/lib/flexnetos 755 flexnet users -"
    "d /var/log/flexnetos 755 flexnet users -"
  ];
}
EOF

# Create modules
cat > "$NIXOS_WSL_DIR/modules/flexnetos-core.nix" << 'EOF'
{ config, lib, pkgs, ... }:

with lib;

{
  options.flexnetos = {
    enable = mkEnableOption "FlexNetOS agentic operating system";
    
    environment = mkOption {
      type = types.enum [ "development" "production" ];
      default = "development";
      description = "FlexNetOS deployment environment";
    };
    
    services = mkOption {
      type = types.attrsOf types.bool;
      default = {
        holochain = true;
        nats = true;
        vault = true;
        kong = true;
        agixt = true;
        localai = true;
        prometheus = true;
        grafana = true;
      };
      description = "Enabled FlexNetOS services";
    };
  };

  config = mkIf config.flexnetos.enable {
    # FlexNetOS system configuration
    systemd.services.flexnetos-init = {
      description = "FlexNetOS initialization service";
      wantedBy = [ "multi-user.target" ];
      
      serviceConfig = {
        Type = "oneshot";
        User = "root";
        ExecStart = "${pkgs.writeShellScript "flexnetos-init" '''
          #!/bin/bash
          set -euo pipefail
          
          echo "Initializing FlexNetOS..."
          
          # Create required directories
          mkdir -p /opt/flexnetos/{agents,config,services}
          mkdir -p /var/lib/flexnetos/{data,cache,logs}
          mkdir -p /etc/flexnetos/{services,secrets}
          
          # Set permissions
          chown -R flexnet:users /opt/flexnetos
          chown -R flexnet:users /var/lib/flexnetos
          
          # Initialize configuration
          if [ ! -f /etc/flexnetos/config.yaml ]; then
            cat > /etc/flexnetos/config.yaml << EOF
# FlexNetOS Configuration
environment: development
version: "1.0.0"

# Core services
services:
  holochain: true
  nats: true
  vault: true
  kong: true
  agixt: true
  localai: true
  prometheus: true
  grafana: true

# Networking
network:
  hostName: flexnetos
  domain: local
  
# Paths
paths:
  data: /var/lib/flexnetos
  config: /etc/flexnetos
  logs: /var/log/flexnetos
EOF
          fi
          
          echo "FlexNetOS initialization complete"
        '''}";
        RemainAfterExit = true;
      };
    };

    # FlexNetOS environment
    environment.etc."flexnetos/version".text = "1.0.0";
    environment.etc."flexnetos/build".text = builtins.getEnv "USER" or "unknown";
    environment.etc."flexnetos/timestamp".text = builtins.toString builtins.currentTime;
  };
}
EOF

echo "NixOS WSL configuration created successfully!"
echo "Location: $NIXOS_WSL_DIR"
echo ""
echo "To build the WSL distribution:"
echo "  1. cd $NIXOS_WSL_DIR"
echo "  2. nix build .#nixosConfigurations.flexnetos-wsl.config.system.build.toplevel"
echo "  3. nix-shell -p gnutar -p gzip --run 'tar -czf flexnetos-rootfs.tar.gz -C result .'"
echo ""
echo "This will create a native NixOS WSL distribution (not Ubuntu+Nix)"
