{
  description = "FlexNetOS - Enterprise Agentic Operating System";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nixos-wsl.url = "github:nix-community/NixOS-WSL";
    flake-utils.url = "github:numtide/flake-utils";
    
    # FlexNetOS specific inputs
    holonix.url = "github:holochain/holonix";
    agixt.url = "github:AGiXT/agixt";
    
    # Additional overlays
    nur.url = "github:nix-community/NUR";
  };

  outputs = { self, nixpkgs, nixos-wsl, flake-utils, holonix, agixt, nur, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
          overlays = [
            nixos-wsl.overlays.default
            agixt.overlays.default
            (import ./nix/overlays/flexnetos.nix)
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
            ./nix/modules/flexnetos-configuration.nix
          ];
          specialArgs = { inherit wslConfig; };
        };

        packages.default = self.nixosConfigurations.flexnetos-wsl.config.system.build.toplevel;
        
        # FlexNetOS specific packages
        packages.flexnetos-config = pkgs.callPackage ./nix/packages/config.nix {};
        packages.flexnetos-cli = pkgs.callPackage ./nix/packages/cli.nix {};
        
        # Development shell
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            nixos-generators
            wslu
            git
            curl
            jq
            yq
            docker-compose
            rustc
            cargo
            go_1_21
            nodejs_20
            python311
          ] ++ (with pkgs.flexnetos; [
            flexnetos-config
            flexnetos-cli
          ]);
          
          shellHook = ''
            echo "Welcome to FlexNetOS Development Environment"
            echo "Version: $(cat ./VERSION)"
            echo "Available services: holochain, nats, vault, kong, agixt, localai, prometheus, grafana"
            echo "Quick start: docker-compose up -d"
          '';
        };

        # Docker image building
        packages.docker-images = pkgs.writeScript "build-docker-images" ''
          #!/usr/bin/env bash
          set -euo pipefail
          
          echo "Building FlexNetOS Docker images..."
          
          # Build each service image
          for service in holochain nats vault kong agixt localai prometheus grafana keycloak minio ipfs tensorzero lobe-chat; do
            echo "Building $service image..."
            docker-compose -f docker/$service.yml build
          done
          
          echo "All FlexNetOS Docker images built successfully!"
        '';

        # Checks and validations
        checks = {
          nixos-config = pkgs.runCommand "check-nixos-config" { } ''
            ${pkgs.nixos-rebuild}/bin/nixos-rebuild dry-activate --flake .#flexnetos-wsl
            touch $out
          '';
          
          docker-configs = pkgs.runCommand "check-docker-configs" { } ''
            for file in ${./docker}/*.yml; do
              ${pkgs.docker-compose}/bin/docker-compose -f "$file" config > /dev/null
            done
            touch $out
          '';
          
          security-audit = pkgs.writeScript "security-audit" ''
            #!/usr/bin/env bash
            echo "Running security audit..."
            ${pkgs.trivy}/bin/trivy fs --security-checks vuln,config .
          '';
        };

        # Deployment automation
        apps.deploy = flake-utils.lib.mkApp {
          drv = pkgs.writeShellScriptBin "deploy-flexnetos" ''
            #!/usr/bin/env bash
            set -euo pipefail
            
            echo "Deploying FlexNetOS..."
            
            # Start core services
            docker-compose -f docker/holochain.yml up -d
            docker-compose -f docker/nats.yml up -d
            docker-compose -f docker/vault.yml up -d
            
            # Wait for services to be ready
            sleep 30
            
            # Start remaining services
            for service in kong agixt localai prometheus grafana keycloak minio ipfs tensorzero lobe-chat; do
              echo "Starting $service..."
              docker-compose -f docker/$service.yml up -d
            done
            
            echo "FlexNetOS deployment completed!"
            echo "Access Grafana: http://localhost:3000"
            echo "Access Prometheus: http://localhost:9090"
            echo "Access Kong Gateway: http://localhost:8000"
          '';
        };

        # NixOS modules for reuse
        nixosModules.flexnetos = ./nix/modules/flexnetos.nix;
        nixosModules.flexnetos-core = ./nix/modules/flexnetos-core.nix;
        nixosModules.flexnetos-services = ./nix/modules/flexnetos-services.nix;
        nixosModules.flexnetos-security = ./nix/modules/flexnetos-security.nix;
        nixosModules.flexnetos-monitoring = ./nix/modules/flexnetos-monitoring.nix;
      });
}
