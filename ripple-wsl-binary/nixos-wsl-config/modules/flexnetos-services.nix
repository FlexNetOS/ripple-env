{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.flexnetos;
  
  # Docker Compose service template
  createDockerService = { name, composeFile, description }: {
    systemd.services."flexnetos-${name}" = {
      inherit description;
      wantedBy = [ "multi-user.target" ];
      after = [ "docker.service" "flexnetos-init.service" ];
      requires = [ "docker.service" ];
      
      serviceConfig = {
        Type = "oneshot";
        RemainAfterExit = true;
        User = "flexnet";
        WorkingDirectory = "/opt/flexnetos";
        ExecStart = "${pkgs.docker-compose}/bin/docker-compose -f ${composeFile} up -d";
        ExecStop = "${pkgs.docker-compose}/bin/docker-compose -f ${composeFile} down";
        Restart = "on-failure";
        RestartSec = 10;
      };
    };
  };

in {
  config = mkIf cfg.enable {
    # Docker configuration for FlexNetOS
    virtualisation.docker = {
      enable = true;
      settings = {
        log-driver = "json-file";
        log-opts = {
          "max-size" = "10m";
          "max-file" = "3";
        };
        
        # FlexNetOS specific settings
        "default-ulimits" = {
          "nofile" = {
            "Hard" = 65536;
            "Soft" = 65536;
          };
        };
        
        "live-restore" = true;
        "userland-proxy" = false;
        
        # Networking
        "bip" = "172.17.0.1/16";
        "default-address-pools" = [
          { "base" = "172.17.0.0/16"; "size" = 24; }
          { "base" = "172.18.0.0/16"; "size" = 24; }
        ];
      };
    };

    # Enable required services
    services.postgresql = {
      enable = mkIf (cfg.services.agixt or false) {
        enable = true;
        package = pkgs.postgresql_15;
        settings = {
          listen_addresses = "127.0.0.1";
          port = 5432;
          max_connections = 100;
          shared_buffers = "256MB";
        };
        
        initialDatabases = [
          { name = "agixt"; }
          { name = "keycloak"; }
          { name = "grafana"; }
        ];
      };
    };

    services.redis = {
      enable = mkIf (cfg.services.agixt or cfg.services.localai or false) {
        enable = true;
        package = pkgs.redis;
        settings = {
          port = 6379;
          bind = "127.0.0.1";
          "maxmemory" = "1gb";
          "maxmemory-policy" = "allkeys-lru";
        };
      };
    };

    # Systemd services for Docker Compose stacks
    systemd.services = mkMerge ([
      (mkIf (cfg.services.holochain or false) (createDockerService {
        name = "holochain";
        composeFile = "/opt/flexnetos/config/docker/holochain.yml";
        description = "Holochain distributed coordination layer";
      }))
      
      (mkIf (cfg.services.nats or false) (createDockerService {
        name = "nats";
        composeFile = "/opt/flexnetos/config/docker/nats.yml";
        description = "NATS event bus messaging";
      }))
      
      (mkIf (cfg.services.vault or false) (createDockerService {
        name = "vault";
        composeFile = "/opt/flexnetos/config/docker/vault.yml";
        description = "HashiCorp Vault secrets management";
      }))
      
      (mkIf (cfg.services.kong or false) (createDockerService {
        name = "kong";
        composeFile = "/opt/flexnetos/config/docker/kong.yml";
        description = "Kong API gateway";
      }))
      
      (mkIf (cfg.services.agixt or false) (createDockerService {
        name = "agixt";
        composeFile = "/opt/flexnetos/config/docker/agixt.yml";
        description = "AGiXT agent orchestration";
      }))
      
      (mkIf (cfg.services.localai or false) (createDockerService {
        name = "localai";
        composeFile = "/opt/flexnetos/config/docker/localai.yml";
        description = "LocalAI inference service";
      }))
      
      (mkIf (cfg.services.prometheus or false) (createDockerService {
        name = "prometheus";
        composeFile = "/opt/flexnetos/config/docker/prometheus.yml";
        description = "Prometheus metrics collection";
      }))
      
      (mkIf (cfg.services.grafana or false) (createDockerService {
        name = "grafana";
        composeFile = "/opt/flexnetos/config/docker/grafana.yml";
        description = "Grafana dashboards";
      }))
    ]);
  };
}
