# Network Requirements

This document describes all network dependencies for ripple-env and provides guidance for operating in restricted network environments.

## Quick Reference

| Service | Required | Purpose | Offline Alternative |
|---------|----------|---------|---------------------|
| cache.nixos.org | Yes | Nix binary cache | Local cache |
| conda.anaconda.org | Yes | Conda packages | Local mirror |
| github.com | Yes* | Flake inputs, git repos | Git bundles |
| docker.io | Optional | Container images | Local registry |
| huggingface.co | Optional | AI models | Pre-downloaded models |

*Required only for initial setup; can be cached for offline use.

## Network Dependencies by Category

### 1. Nix Binary Caches

Nix downloads pre-built packages from binary caches to avoid local compilation.

**Required Endpoints:**

| Host | Port | Protocol | Purpose |
|------|------|----------|---------|
| cache.nixos.org | 443 | HTTPS | Official NixOS binary cache |
| nix-community.cachix.org | 443 | HTTPS | Community packages |
| cuda-maintainers.cachix.org | 443 | HTTPS | CUDA packages (GPU only) |

**Bandwidth Estimate:**
- Initial setup: 2-5 GB
- Updates: 100-500 MB per update

**Offline Configuration:**
```bash
# Use local cache only
export NIX_CONFIG="substituters = file:///var/cache/ripple-env/nix-store"
```

### 2. Conda/Pixi Channels

Pixi downloads packages from Conda channels for ROS2 and Python dependencies.

**Required Endpoints:**

| Host | Port | Protocol | Purpose |
|------|------|----------|---------|
| conda.anaconda.org | 443 | HTTPS | Main Conda repository |

**Channels Used:**
- `robostack-humble` - ROS2 Humble packages
- `conda-forge` - General packages
- `pytorch` - PyTorch (CUDA feature only)
- `nvidia` - NVIDIA packages (CUDA feature only)

**Bandwidth Estimate:**
- Initial setup: 3-8 GB (depends on features)
- Updates: 200-800 MB per update

**Offline Configuration:**
```toml
# ~/.config/pixi/config.toml
[mirrors]
"https://conda.anaconda.org/conda-forge" = ["file:///var/cache/ripple-env/conda-channels/conda-forge"]
```

### 3. GitHub Repositories

GitHub hosts flake inputs, source code, and release artifacts.

**Required Endpoints:**

| Host | Port | Protocol | Purpose |
|------|------|----------|---------|
| github.com | 443 | HTTPS | Repository access |
| api.github.com | 443 | HTTPS | API (releases, actions) |
| raw.githubusercontent.com | 443 | HTTPS | Raw file access |
| objects.githubusercontent.com | 443 | HTTPS | Git LFS objects |

**Repositories Accessed:**
- `nixos/nixpkgs` - Nix packages
- `nix-community/home-manager` - User configuration
- `nix-community/NixOS-WSL` - WSL2 support
- `FlexNetOS/ripple-env` - This repository
- Various action runners and tools

**Offline Alternative:**
Use git bundles or a local git mirror:
```bash
# Create bundle on connected machine
git clone --mirror https://github.com/nixos/nixpkgs.git
git bundle create nixpkgs.bundle --all

# Use bundle on air-gapped machine
git clone nixpkgs.bundle nixpkgs
```

### 4. Docker Container Registries

Container images are pulled from multiple registries.

**Registry Endpoints:**

| Registry | Host | Purpose |
|----------|------|---------|
| Docker Hub | registry-1.docker.io | Main container registry |
| GitHub Packages | ghcr.io | GitHub container registry |
| Quay.io | quay.io | Red Hat container registry |
| Google GCR | gcr.io | Google container registry |

**Images Used (Core):**
- `redis:7-alpine` - Caching
- `postgres:17.2-alpine` - Database
- `nats:2.10.24-alpine` - Messaging

**Images Used (Extended):**
- `localai/localai:*` - AI inference
- `temporalio/*` - Workflow engine
- `grafana/*` - Monitoring
- `hashicorp/vault:*` - Secrets
- `quay.io/keycloak/*` - Identity

See `config/offline.yaml` for complete image list.

**Offline Configuration:**
```bash
# Start local registry
docker run -d -p 5000:5000 --name registry registry:2

# Configure Docker daemon (/etc/docker/daemon.json)
{
  "registry-mirrors": ["http://localhost:5000"],
  "insecure-registries": ["localhost:5000"]
}
```

### 5. AI Model Downloads

AI services download models from HuggingFace and other sources.

**Endpoints:**

| Host | Port | Protocol | Purpose |
|------|------|----------|---------|
| huggingface.co | 443 | HTTPS | Model hub |
| cdn-lfs.huggingface.co | 443 | HTTPS | Large file storage |

**Models Used:**
- Mistral 7B (Q4_K_M quantized) - ~4 GB
- Other models as configured in LocalAI

**Bandwidth Estimate:**
- Per model: 1-10 GB depending on size

**Offline Configuration:**
```bash
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export HF_HOME=/var/cache/ripple-env/models/huggingface
```

### 6. CI/CD and GitHub Actions

GitHub Actions workflows require network access for runners and actions.

**Endpoints:**

| Host | Purpose |
|------|---------|
| github.com | Action repository |
| actions-results*.actions.githubusercontent.com | Results storage |
| pipelines.actions.githubusercontent.com | Pipeline API |
| vstoken.actions.githubusercontent.com | Token service |

**Self-Hosted Runners:**
For air-gapped CI/CD, use self-hosted runners with pre-cached actions.

## Firewall Rules

### Minimum Required (Outbound)

```
# Nix binary caches
ALLOW TCP 443 -> cache.nixos.org
ALLOW TCP 443 -> nix-community.cachix.org

# Conda channels
ALLOW TCP 443 -> conda.anaconda.org

# GitHub (for flake inputs)
ALLOW TCP 443 -> github.com
ALLOW TCP 22 -> github.com  # For SSH git access
```

### Extended (Full Features)

```
# Docker registries
ALLOW TCP 443 -> registry-1.docker.io
ALLOW TCP 443 -> ghcr.io
ALLOW TCP 443 -> quay.io

# AI models
ALLOW TCP 443 -> huggingface.co
ALLOW TCP 443 -> cdn-lfs.huggingface.co

# CUDA support
ALLOW TCP 443 -> cuda-maintainers.cachix.org
```

## Offline Installation

### Preparation (Connected Machine)

```bash
# Clone repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# Prepare offline packages
./scripts/prepare-offline.sh --all --full

# This creates: ripple-env-offline-full-YYYYMMDD.tar.gz
```

### Deployment (Air-Gapped Machine)

```bash
# Transfer the bundle and extract
tar -xzf ripple-env-offline-full-*.tar.gz -C /var/cache/

# Deploy
cd ripple-env
./scripts/deploy-airgapped.sh --bundle /path/to/bundle.tar.gz

# Activate offline mode
source ~/.config/ripple-env/offline.env

# Start development
nix develop
```

## Mirror Configuration

### Enterprise Mirrors

For enterprise environments, configure mirrors in `~/.config/ripple-env/mirrors.yaml`:

```yaml
nix:
  private_substituters:
    - url: "https://nix-cache.corp.example.com"
      priority: 10
      public_key: "corp-cache:xxxx"

pixi:
  channels:
    conda-forge:
      mirrors:
        - "https://artifactory.corp.example.com/conda-forge"

docker:
  registry_mirrors:
    - "https://docker-mirror.corp.example.com"
```

### Proxy Configuration

For environments requiring a proxy:

```bash
export HTTP_PROXY="http://proxy.example.com:8080"
export HTTPS_PROXY="http://proxy.example.com:8080"
export NO_PROXY="localhost,127.0.0.1,.local,.corp.example.com"
```

## Bandwidth Optimization

### Reduce Initial Download Size

1. **Use minimal shell:**
   ```bash
   nix develop .#minimal
   ```

2. **Disable optional features:**
   ```bash
   # Skip CUDA packages
   export RIPPLE_NO_CUDA=1
   ```

3. **Use specific platform:**
   Only download packages for your platform (linux-64, osx-arm64, etc.)

### Caching Strategies

1. **Shared Nix store:**
   Mount a shared network drive for `/nix/store`

2. **Binary cache server:**
   Run `nix-serve` or `attic` for team caching

3. **Pixi cache sharing:**
   ```bash
   export PIXI_CACHE_DIR=/shared/cache/pixi
   ```

## Troubleshooting

### Connection Timeouts

```bash
# Increase Nix timeout
export NIX_CONFIG="connect-timeout = 30"

# Retry with fallback
nix build --fallback
```

### SSL Certificate Issues

```bash
# Use system certificates
export NIX_SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

# For corporate CAs
export CURL_CA_BUNDLE=/path/to/corporate-ca.crt
```

### Partial Downloads

```bash
# Resume Nix downloads
nix build --keep-going

# Clear and retry Pixi
pixi clean
pixi install
```

## Verification

Test network requirements are met:

```bash
# Check Nix cache connectivity
curl -sI https://cache.nixos.org/nix-cache-info

# Check Conda channel
curl -sI https://conda.anaconda.org/conda-forge/linux-64/repodata.json

# Check GitHub API
curl -sI https://api.github.com/rate_limit

# Verify Docker registry
docker pull hello-world
```

## See Also

- [Getting Started Guide](GETTING_STARTED.md)
- [Troubleshooting](TROUBLESHOOTING.md)
- [Supply Chain Security](SUPPLY_CHAIN_SECURITY.md)
- [config/offline.yaml](../config/offline.yaml) - Offline configuration
- [config/mirrors.yaml](../config/mirrors.yaml) - Mirror configuration
