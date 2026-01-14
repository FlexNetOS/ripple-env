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
        ExecStart = pkgs.writeShellScript "flexnetos-init" ''
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
            cat > /etc/flexnetos/config.yaml << 'EOF'
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
        '';
        RemainAfterExit = true;
      };
    };

    # FlexNetOS environment
    environment.etc."flexnetos/version".text = "1.0.0";
    environment.etc."flexnetos/build".text = builtins.getEnv "USER" or "unknown";
    environment.etc."flexnetos/timestamp".text = builtins.toString builtins.currentTime;
  };
}
