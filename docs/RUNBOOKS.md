# Runbooks & Cookbooks

**Status:** Phase 7 Complete
**Last Updated:** 2026-01-14
**Evidence:** scripts, docker-compose files, CI workflows

---

## Quick Reference

| Task | Command | Section |
|------|---------|---------|
| Bootstrap fresh machine | `./bootstrap.sh` or `.\bootstrap.ps1` | [Bootstrap](#runbook-bootstrap) |
| Enter dev environment | `nix develop` or `nom develop` | [Development](#runbook-development) |
| Build ROS2 packages | `cb` (colcon build) | [Build](#runbook-build) |
| Deploy full stack | `./scripts/deploy.sh` | [Deploy](#runbook-deploy) |
| Run E2E validation | `./scripts/validate-e2e.sh` | [Validate](#runbook-validate) |
| Security audit | `./scripts/security-audit.sh` | [Security](#runbook-security) |
| Rotate certificates | `./scripts/rotate-certs.sh` | [Certificate](#runbook-certificates) |
| Cleanup WSL | `./scripts/wsl-cleanup.sh` | [Maintenance](#runbook-maintenance) |

---

## Runbook: Bootstrap

### Linux/macOS Fresh Install

**Purpose:** Set up development environment from scratch
**Time:** 15-30 minutes
**Prerequisites:** curl, internet access

```bash
# 1. Download bootstrap script
curl -fsSL https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.sh -o bootstrap.sh

# 2. Make executable
chmod +x bootstrap.sh

# 3. Run bootstrap
./bootstrap.sh

# 4. Enter development shell
cd ripple-env
nix develop

# 5. Verify
ros2 --help
pixi --version
```

**Flags:**
- `--ci` — Non-interactive mode
- `--skip-shells` — Skip zsh/nushell installation
- `--verify` — Run verification after install
- `--resume` — Resume from failed state
- `--clean` — Reset state and start fresh

**Troubleshooting:**
- If Nix fails: Check `~/.nix-defexpr/` permissions
- If direnv fails: Run `eval "$(direnv hook bash)"`
- If pixi fails: Delete `.pixi/` and retry

---

### Windows Fresh Install

**Purpose:** Set up WSL2 + NixOS environment
**Time:** 30-60 minutes
**Prerequisites:** Windows 10 Build 19041+, Admin rights

```powershell
# 1. Open PowerShell as Administrator

# 2. Download bootstrap script
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.ps1" -OutFile "bootstrap.ps1"

# 3. Run bootstrap
.\bootstrap.ps1

# 4. Enter WSL
wsl -d NixOS-Ripple

# 5. Enter development shell
cd ~/ripple-env
nix develop
```

**Flags:**
- `-DistroName` — Custom WSL distro name (default: NixOS-Ripple)
- `-DiskSizeGB` — Virtual disk size (default: 1000)
- `-MemorySizeGB` — WSL memory limit (default: 8)
- `-Resume` — Resume from failed state
- `-Clean` — Reset and start fresh

**Troubleshooting:**
- WSL won't start: Run `wsl --update`
- Disk space: Run `./scripts/wsl-cleanup.sh` inside WSL
- Memory issues: Edit `~/.wslconfig` to increase memory

---

## Runbook: Development

### Daily Development Workflow

```bash
# 1. Enter development shell (pick one)
nix develop          # Standard
nom develop          # With progress bar
direnv allow         # Auto-activate on cd

# 2. Update environment (if needed)
git pull
nix flake update
pixi update

# 3. Start services (if needed)
docker compose -f docker/docker-compose.observability.yml up -d

# 4. Work on code...

# 5. Build and test
cb                   # colcon build
ct                   # colcon test
ctr                  # colcon test-result --verbose
```

### Adding a New ROS2 Package

```bash
# 1. Create package directory
mkdir -p ~/ros2_ws/src/my_package

# 2. Initialize package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_package

# 3. Build
cd ~/ros2_ws
cb

# 4. Source
source install/setup.bash

# 5. Run
ros2 run my_package my_node
```

---

## Runbook: Build

### Build All ROS2 Packages

```bash
cd ~/ros2_ws

# Full build
colcon build --symlink-install

# Or use alias
cb
```

### Build Specific Package

```bash
# Single package
colcon build --packages-select my_package

# Package and dependencies
colcon build --packages-up-to my_package
```

### Clean Build

```bash
# Remove build artifacts
rm -rf build/ install/ log/

# Rebuild
cb
```

### Build Frontend

```bash
./scripts/build-frontend.sh
```

---

## Runbook: Deploy

### Deploy Full Stack

```bash
# 1. Initialize infrastructure
./scripts/init-docker-networks.sh
./scripts/init-step-ca.sh
./scripts/init-jetstream.sh

# 2. Deploy main stack
./scripts/deploy.sh

# 3. Deploy observability
./scripts/deploy-observability.sh

# 4. Verify
./scripts/validate-e2e.sh
```

### Deploy Individual Stacks

```bash
# Observability only
docker compose -f docker/docker-compose.observability.yml up -d

# Messaging only
docker compose -f docker/docker-compose.messaging.yml up -d

# AI services only
docker compose -f docker/docker-compose.localai.yml up -d
docker compose -f docker/docker-compose.agixt.yml up -d
```

### Deploy to Kubernetes

```bash
# 1. Install ArgoCD
./scripts/install-argocd.sh

# 2. Install Argo Rollouts
./scripts/install-argo-rollouts.sh

# 3. Apply manifests
kubectl apply -k manifests/

# 4. Sync via ArgoCD
argocd app sync ripple-env
```

### Airgapped Deployment

```bash
# 1. Prepare offline package
./scripts/prepare-offline.sh

# 2. Copy to airgapped environment
# (manual transfer)

# 3. Deploy from local images
./scripts/deploy-airgapped.sh
```

---

## Runbook: Validate

### End-to-End Validation

```bash
./scripts/validate-e2e.sh

# Expected output:
# Phase 1: Configuration Validation
# Phase 2: Package Verification
# Phase 3: Docker Compose Structure Validation
# Phase 4: OPA Policy Validation
# Phase 5: Script Validation
# Phase 6: Documentation Check
#
# RESULT: PASS
```

### Component Verification

```bash
# All components
./scripts/verify-argo-workflows.sh
./scripts/verify-observability.sh
./scripts/verify-jetstream.sh
./scripts/verify-mindsdb.sh
./scripts/verify-mtls-setup.sh
./scripts/verify-open-lovable.sh
./scripts/verify-qudag.sh
./scripts/verify-ruvector.sh
./scripts/verify-state-storage.sh
./scripts/verify-edge.sh
```

### Configuration Validation

```bash
# Validate all configs
./scripts/validate-configs.sh

# Validate ARIA manifest
python scripts/validate-manifest.py --profile ci

# Validate Pixi channels
python scripts/validate-channels.py
```

---

## Runbook: Security

### Security Audit

```bash
# Full audit
./scripts/security-audit.sh

# Require all tools
SECURITY_AUDIT_REQUIRE_TRIVY=1 ./scripts/security-audit.sh

# Git history scan
SECURITY_AUDIT_GITLEAKS_MODE=git ./scripts/security-audit.sh
```

### Container Scanning

```bash
./scripts/scan-containers.sh
```

### SBOM Generation

```bash
# Generate SBOM
syft . -o spdx-json > sbom.json

# Vulnerability scan
grype sbom:./sbom.json
```

---

## Runbook: Certificates

### Initialize PKI

```bash
# First-time setup
./scripts/init-step-ca.sh

# Check CA status
step ca health
```

### Generate Service Certificates

```bash
./scripts/generate-service-certs.sh

# Certificates created:
# secrets/certs/ca.crt
# secrets/certs/server.crt
# secrets/certs/server.key
# secrets/certs/client.crt
# secrets/certs/client.key
```

### Rotate Certificates

```bash
# Manual rotation
./scripts/rotate-certs.sh

# Setup automatic rotation
./scripts/setup-cert-rotation-cron.sh
```

### Verify mTLS

```bash
./scripts/verify-mtls-setup.sh
```

---

## Runbook: Maintenance

### Update Dependencies

```bash
# Nix packages
nix flake update

# Pixi packages
pixi update

# Python packages (check only)
./scripts/check-python-deps.sh

# Python packages (upgrade)
./scripts/upgrade-python-deps.sh
```

### Cleanup WSL

```bash
# From inside WSL
./scripts/wsl-cleanup.sh

# From Windows PowerShell
.\scripts\Cleanup-WSL.ps1
```

### Fix WSL Stability

```bash
./scripts/fix-wsl-stability.sh
```

### Garbage Collection

```bash
# Nix garbage collection
nix-collect-garbage -d

# Docker cleanup
docker system prune -af
docker volume prune -f
```

---

## Runbook: Troubleshooting

### Common Issues

#### Nix Flake Check Fails

```bash
# Update flake inputs
nix flake update

# Check for errors
nix flake check --show-trace

# Fix common issues
nix-collect-garbage -d
rm -rf ~/.cache/nix
```

#### Pixi Install Fails

```bash
# Reset pixi environment
rm -rf .pixi/ pixi.lock

# Reinstall
pixi install
```

#### Docker Services Won't Start

```bash
# Check logs
docker compose logs <service>

# Check port conflicts
sudo lsof -i :<port>

# Reset Docker
docker compose down -v
docker compose up -d
```

#### ROS2 Node Can't Find Messages

```bash
# Rebuild messages
cd ~/ros2_ws
rm -rf build/ install/
cb

# Source again
source install/setup.bash
```

#### Out of Disk Space

```bash
# Check usage
df -h

# Cleanup Nix
nix-collect-garbage -d

# Cleanup Docker
docker system prune -af

# Cleanup pixi
rm -rf .pixi/
pixi install
```

---

## Cookbooks

### Cookbook: Add New AI Model to LocalAI

```bash
# 1. Download model
./scripts/download-models.sh llama-2-7b

# 2. Restart LocalAI
docker compose -f docker/docker-compose.localai.yml restart

# 3. Verify model available
curl http://localhost:8080/v1/models
```

### Cookbook: Create New Keycloak Realm

```bash
# 1. Access Keycloak admin
open http://localhost:8080/admin

# 2. Login (admin / <password>)

# 3. Create realm via UI or:
curl -X POST http://localhost:8080/admin/realms \
  -H "Authorization: Bearer $KEYCLOAK_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"realm": "my-realm", "enabled": true}'
```

### Cookbook: Add Prometheus Scrape Target

```yaml
# config/prometheus/prometheus.yml
scrape_configs:
  - job_name: 'my-service'
    static_configs:
      - targets: ['my-service:8080']
    metrics_path: '/metrics'
```

```bash
# Reload config
docker compose -f docker/docker-compose.observability.yml restart prometheus
```

### Cookbook: Create Vault Secret

```bash
# Enable KV secrets engine (once)
vault secrets enable -path=secret kv-v2

# Store secret
vault kv put secret/myapp api_key="secret123"

# Read secret
vault kv get secret/myapp

# Delete secret
vault kv delete secret/myapp
```

### Cookbook: Add NATS Stream

```bash
# Create stream
nats stream add MY_STREAM \
  --subjects "my.subject.>" \
  --storage file \
  --retention limits \
  --max-msgs 1000000

# Create consumer
nats consumer add MY_STREAM MY_CONSUMER \
  --filter "my.subject.>" \
  --ack explicit
```

---

## Emergency Procedures

### Service Recovery

```bash
# 1. Stop all services
docker compose down

# 2. Check system resources
df -h
free -m

# 3. Clear Docker
docker system prune -af

# 4. Restart services
./scripts/deploy.sh
```

### Rollback Deployment

```bash
# ArgoCD rollback
argocd app rollback ripple-env <revision>

# Docker rollback (if using tags)
docker compose down
docker compose -f docker-compose.yml.backup up -d
```

### Restore from Backup

```bash
# Database restore
pg_restore -d ripple /backups/ripple-$(date +%Y%m%d).dump

# MinIO restore
mc mirror /backups/minio/ minio/bucket/
```
