# Manifests Directory

This directory contains declarative configuration manifests for the FlexStack agentic OS.

## Quick Reference

| File | Purpose |
|------|---------|
| `stack.yaml` | SSoT pointer file - maps all configuration sources |
| `build_phases.json` | 7-phase build orchestration |
| `feature_flags.yaml` | Feature flag definitions |

## Directory Structure (per BUILDKIT_STARTER_SPEC.md §12)

```
manifests/
├── stack.yaml              # SSoT pointer file
├── build_phases.json       # Build phase orchestration
├── feature_flags.yaml      # Feature flags
│
├── wsl2/                   # Windows/WSL2 configuration
│   ├── distro.json         # NixOS-WSL distribution config
│   └── assets.json         # Release artifacts & binaries
│
├── cloud/                  # Cloud/CI configuration
│   ├── builder_image.json  # Builder image configs
│   ├── ci_pipeline.yaml    # CI/CD pipeline definition
│   └── mcp_tools.json      # MCP tool configurations
│
├── distributed/            # Distributed orchestration
│   ├── compute_policy.yaml
│   ├── inference_policy.yaml
│   ├── memory_policy.yaml
│   ├── resource_policy.yaml
│   └── storage_policy.yaml
│
├── holochain/              # P2P coordination
│   ├── conductor.yaml
│   ├── networks.json
│   ├── versions.json
│   └── dnas/
│
├── observability/          # Monitoring configuration
│   ├── prometheus.yml
│   ├── loki.yml
│   └── grafana/
│
├── llmops/                 # LLM operations
│   └── tensorzero.toml
│
├── argo-workflows/         # Workflow templates
│   └── templates/
│
├── argocd/                 # GitOps applications
│   └── apps/
│
├── temporal/               # Temporal workflows
│   └── namespaces/
│
├── mcp/                    # MCP tool schemas
│   └── schemas/
│
└── capability-registry/    # API capabilities
    └── registry.json
```

## Build Phases

The `build_phases.json` defines 7 sequential phases executed by `scripts/install-all.sh`:

| Phase | Name | Description | Platforms |
|-------|------|-------------|-----------|
| 0 | Preflight | Validate dependencies (nix, pixi, docker) | All |
| 1 | Environment | Configure direnv, set env vars | All |
| 2 | Nix Packages | Build development shell | All |
| 3 | Pixi Packages | Install Python, ROS2, ML packages | All |
| 4 | Docker Network | Create agentic-network | Linux, WSL2 |
| 5 | Services | Start Docker Compose stacks | Linux, WSL2 |
| 6 | Kubernetes | Optional Argo CD setup | Linux, WSL2 |
| 7 | Verification | Run ARIA manifest verification | All |

### Execution Profiles

```bash
# Minimal - Core tools only (fastest)
./scripts/install-all.sh --profile minimal

# CI - No services, non-interactive
./scripts/install-all.sh --profile ci

# Default - Standard development
./scripts/install-all.sh --profile default

# Full - Everything including K8s
./scripts/install-all.sh --profile full
```

## Cross-Platform Support

### Linux

```bash
# Bootstrap
./bootstrap.sh

# Full installation
./scripts/install-all.sh
```

### macOS

```bash
# Bootstrap (Docker services run in Docker Desktop)
./bootstrap.sh

# Installation (skips Docker phases)
./scripts/install-all.sh
```

### Windows/WSL2

```powershell
# Run from PowerShell (Administrator)
.\bootstrap.ps1

# Customizations
.\bootstrap.ps1 -DistroName "MyNixOS" -DiskSizeGB 512 -MemorySizeGB 16
```

Configuration is stored in `manifests/wsl2/`:
- `distro.json` - NixOS-WSL distribution settings
- `assets.json` - Release binaries and artifacts

### Cloud/CI

Cloud configurations in `manifests/cloud/`:
- `builder_image.json` - CI builder image configs
- `ci_pipeline.yaml` - Pipeline stage definitions
- `mcp_tools.json` - MCP tool configurations

## Service Stacks

Phase 5 starts Docker Compose stacks in dependency order:

```
1. Observability (Prometheus, Grafana, Loki)
2. Messaging (NATS, Temporal)
3. Automation (n8n)
4. Edge (Kong)
5. Inference (LocalAI)
6. UI (Lobe Chat, Open-Lovable)
7. LLMOps (TensorZero, MLflow)
```

Start individual stacks:

```bash
# Start observability
docker compose -f docker/docker-compose.observability.yml up -d

# Start messaging
docker compose -f docker/docker-compose.messaging.yml up -d

# Check health
curl http://localhost:9090/-/healthy  # Prometheus
curl http://localhost:3000/api/health # Grafana
curl http://localhost:8222/healthz    # NATS
```

## Verification

```bash
# Verify against ARIA manifest
python scripts/verify-manifest.py --profile default

# Verify components
./scripts/verify-components.sh --profile default

# Full verification
./scripts/verify-components.sh --profile full
```

## Related Documentation

- [BUILDKIT_STARTER_SPEC.md](../BUILDKIT_STARTER_SPEC.md) - Authoritative stack specification
- [ARIA_MANIFEST.yaml](../ARIA_MANIFEST.yaml) - Component verification definitions
- [README.md](../README.md) - Project overview
- [docs/adr/](../docs/adr/) - Architecture Decision Records
