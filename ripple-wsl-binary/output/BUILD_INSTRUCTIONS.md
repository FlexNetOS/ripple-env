# FlexNetOS NixOS WSL Build Instructions

## Prerequisites
- Nix package manager installed
- 8GB+ RAM
- 20GB+ free disk space
- Internet connection

## Build Steps

1. **Navigate to configuration directory:**
   ```bash
   cd /mnt/okcomputer/output/flexnetos-wsl-binary/nixos-wsl-config
   ```

2. **Update flake inputs:**
   ```bash
   nix flake update
   ```

3. **Build the system:**
   ```bash
   nix build .#nixosConfigurations.flexnetos-wsl.config.system.build.toplevel
   ```

4. **Create WSL rootfs:**
   ```bash
   nix-shell -p gnutar -p gzip --run "tar -czf ../output/flexnetos-rootfs.tar.gz -C result ."
   ```

## Installation

1. **Import into WSL:**
   ```powershell
   wsl --import FlexNetOS $env:LOCALAPPDATA\Packages\FlexNetOS flexnetos-rootfs.tar.gz --version 2
   ```

2. **Start FlexNetOS:**
   ```bash
   wsl -d FlexNetOS
   ```

## Features

- Native NixOS (not Ubuntu+Nix)
- Systemd init system
- Docker containerization
- All 13 BUILDKIT services
- Enterprise security
- Monitoring and observability
- Health checks

## Services Included

- Holochain (P2P coordination)
- NATS (Event bus)
- Vault (Secrets management)
- Kong (API Gateway)
- AGiXT (Agent orchestration)
- LocalAI (Inference)
- Prometheus (Metrics)
- Grafana (Dashboards)
- Keycloak (Identity)
- MinIO (Object storage)
- IPFS (Distributed storage)
- TensorZero (LLMOps)
- Lobe Chat (UI)

Build time: ~30-60 minutes
Distribution size: ~2-3GB compressed
