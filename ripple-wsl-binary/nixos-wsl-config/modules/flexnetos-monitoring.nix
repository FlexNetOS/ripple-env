{ config, lib, pkgs, ... }:

with lib;

{
  config = mkIf config.flexnetos.enable {
    # Prometheus configuration
    services.prometheus = {
      enable = mkIf (config.flexnetos.services.prometheus or false) {
        enable = true;
        port = 9090;
        listenAddress = "127.0.0.1";
        
        scrapeConfigs = [
          {
            job_name = "flexnetos";
            static_configs = [
              { targets = [ "localhost:9100" ]; } # Node exporter
              { targets = [ "localhost:9090" ]; } # Prometheus itself
            ];
          }
          {
            job_name = "docker";
            static_configs = [
              { targets = [ "localhost:9323" ]; } # Docker metrics
            ];
          }
        ];
        
        alertmanagerURL = [ "http://localhost:9093" ];
      };
    };

    # Node exporter
    services.prometheus.exporters.node = {
      enable = mkIf (config.flexnetos.services.prometheus or false) {
        enable = true;
        port = 9100;
        listenAddress = "127.0.0.1";
        enabledCollectors = [
          "systemd"
          "textfile"
          "filesystem"
          "meminfo"
          "cpu"
          "diskstats"
          "netstat"
          "filefd"
          "vmstat"
        ];
      };
    };

    # Grafana
    services.grafana = {
      enable = mkIf (config.flexnetos.services.grafana or false) {
        enable = true;
        settings = {
          server = {
            http_addr = "127.0.0.1";
            http_port = 3000;
            domain = "flexnetos.local";
          };
          
          database = {
            type = "sqlite3";
            path = "/var/lib/flexnetos/grafana.db";
          };
          
          security = {
            admin_user = "admin";
            admin_password = "flexnetos-admin";
            secret_key = "flexnetos-secret-key";
          };
          
          dashboards = {
            default_home_dashboard_path = "/etc/flexnetos/grafana/dashboards/flexnetos-overview.json";
          };
        };
        
        provision = {
          enable = true;
          datasources = [
            {
              name = "prometheus";
              type = "prometheus";
              url = "http://localhost:9090";
              isDefault = true;
              editable = true;
            }
          ];
          
          dashboards = [
            {
              name = "flexnetos";
              type = "file";
              options.path = "/etc/flexnetos/grafana/dashboards";
            }
          ];
        };
      };
    };

    # Health check service
    systemd.services.flexnetos-health = {
      description = "FlexNetOS health monitoring";
      wantedBy = [ "multi-user.target" ];
      startAt = "*:*:0/30"; # Every 30 seconds
      
      serviceConfig = {
        Type = "oneshot";
        User = "flexnet";
        ExecStart = pkgs.writeShellScript "flexnetos-health-check" ''
          #!/bin/bash
          set -euo pipefail
          
          HEALTH_FILE="/var/lib/flexnetos/health.json"
          TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
          
          # Check services
          declare -A services=(
            ["docker"]="systemctl is-active docker"
            ["postgresql"]="systemctl is-active postgresql"
            ["redis"]="systemctl is-active redis"
            ["prometheus"]="systemctl is-active prometheus"
            ["grafana"]="systemctl is-active grafana"
          )
          
          echo "{\"timestamp\":\"$TIMESTAMP\",\"services\":{" > "$HEALTH_FILE.tmp"
          
          first=true
          for service in "''${!services[@]}"; do
            if [ "$first" = false ]; then
              echo "," >> "$HEALTH_FILE.tmp"
            fi
            first=false
            
            if eval "''${services[$service]}" >/dev/null 2>&1; then
              echo "\"$service\":\"healthy\"" >> "$HEALTH_FILE.tmp"
            else
              echo "\"$service\":\"unhealthy\"" >> "$HEALTH_FILE.tmp"
            fi
          done
          
          echo "}}" >> "$HEALTH_FILE.tmp"
          mv "$HEALTH_FILE.tmp" "$HEALTH_FILE"
          
          # Log health status
          logger -t flexnetos-health "Health check completed"
        '';
      };
    };

    # Log rotation for FlexNetOS
    services.logrotate.settings."/var/log/flexnetos/*.log" = {
      frequency = "daily";
      rotate = 7;
      compress = true;
      delaycompress = true;
      missingok = true;
      notifempty = true;
      create = "644 flexnet users";
    };
  };
}
