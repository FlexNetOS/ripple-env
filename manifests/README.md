# Manifests Directory

This directory contains declarative configuration manifests for the FlexStack agentic OS.

## Structure (per BUILDKIT_STARTER_SPEC.md §12)

### Implemented

| Directory/File | Description | Status |
|----------------|-------------|--------|
| `distributed/` | Distributed orchestration policies (compute, inference, storage, memory) | Implemented |
| `holochain/` | Holochain configuration (conductor, networks, DNAs) | Implemented |
| `observability/` | Prometheus, Loki, Grafana, OTel configurations | Implemented |
| `mcp/` | MCP tool schemas and documentation | Implemented |
| `argo-workflows/` | Workflow templates and RBAC | Implemented |
| `argocd/` | GitOps application definitions | Implemented |
| `temporal/` | Temporal workflow configurations | Implemented |
| `llmops/` | TensorZero configuration | Implemented |
| `capability-registry/` | API capability registry | Implemented |
| `feature_flags.yaml` | Feature flag definitions | Implemented |

### Planned (per BUILDKIT_STARTER_SPEC.md §12)

| Directory/File | Description | Priority |
|----------------|-------------|----------|
| `build_phases.json` | Build phase ordering for WSL2 binary | P2 |
| `stack.yaml` | Source of truth pointers | P2 |
| `task_graph.json` | Task dependencies graph | P2 |
| `task_graph.mmd` | Mermaid visualization | P2 |
| `vendor.lock.json` | Vendor locks | P2 |
| `wsl2/` | WSL2 distro configuration (distro.json, assets.json) | P2 |
| `cloud/` | Cloud build configuration (builder_image, mcp_tools, ci_pipeline) | P3 |

## Usage

### Distributed Policies

```bash
# View inference policy
cat manifests/distributed/inference_policy.yaml

# View compute policy
cat manifests/distributed/compute_policy.yaml
```

### Holochain Configuration

```bash
# View conductor config
cat manifests/holochain/conductor.yaml

# View network bootstrap servers
cat manifests/holochain/networks.json

# View DNA definitions
ls manifests/holochain/dnas/
```

### Observability

```bash
# Deploy observability stack
docker compose -f docker/docker-compose.observability.yml up -d

# Prometheus config is in manifests/observability/prometheus.yml
```

### Argo Workflows

```bash
# Apply workflow templates
kubectl apply -f manifests/argo-workflows/templates/

# View available templates
ls manifests/argo-workflows/templates/
```

## Related Documentation

- [BUILDKIT_STARTER_SPEC.md](../BUILDKIT_STARTER_SPEC.md) - SSoT for stack architecture
- [ARIA_MANIFEST.yaml](../ARIA_MANIFEST.yaml) - Component verification definitions
- [docs/adr/](../docs/adr/) - Architecture Decision Records
