# FlexStack Helm Charts

This directory contains Helm charts for deploying FlexStack on Kubernetes.

## Charts

| Chart | Description |
|-------|-------------|
| [flexstack](./flexstack/) | Main umbrella chart for deploying the complete FlexStack platform |

## Prerequisites

- Kubernetes 1.25+
- Helm 3.10+
- PV provisioner support in the underlying infrastructure
- kubectl configured with cluster access

## Quick Start

### Add Helm Repositories (for dependencies)

```bash
# Add required Helm repositories
helm repo add bitnami https://charts.bitnami.com/bitnami
helm repo add grafana https://grafana.github.io/helm-charts
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm repo add kong https://charts.konghq.com
helm repo add nats https://nats-io.github.io/k8s/helm/charts
helm repo update
```

### Install FlexStack

```bash
# Install with default values
helm install flexstack ./flexstack

# Install with custom values
helm install flexstack ./flexstack -f custom-values.yaml

# Install in a specific namespace
helm install flexstack ./flexstack --namespace flexstack --create-namespace
```

### Upgrade

```bash
helm upgrade flexstack ./flexstack -f custom-values.yaml
```

### Uninstall

```bash
helm uninstall flexstack
```

## Configuration

See [flexstack/values.yaml](./flexstack/values.yaml) for the full list of configurable parameters.

### Key Configuration Options

| Parameter | Description | Default |
|-----------|-------------|---------|
| `global.storageClass` | Storage class for PVCs | `""` |
| `global.domain` | Domain for ingress | `flexstack.local` |
| `postgresql.enabled` | Enable PostgreSQL | `true` |
| `redis.enabled` | Enable Redis | `true` |
| `minio.enabled` | Enable MinIO | `true` |
| `ai.localai.enabled` | Enable LocalAI | `true` |
| `ai.agixt.enabled` | Enable AGiXT | `true` |
| `observability.enabled` | Enable observability stack | `true` |
| `messaging.temporal.enabled` | Enable Temporal | `true` |

### Example Custom Values

```yaml
# production-values.yaml
global:
  storageClass: fast-ssd
  domain: flexstack.example.com

postgresql:
  primary:
    persistence:
      size: 100Gi

ai:
  localai:
    replicas: 2
    resources:
      limits:
        memory: 16Gi
        cpu: "8"

ingress:
  enabled: true
  className: nginx
  annotations:
    cert-manager.io/cluster-issuer: letsencrypt
  tls:
    - secretName: flexstack-tls
      hosts:
        - flexstack.example.com
```

## Service Profiles

FlexStack services are organized into profiles, similar to Docker Compose:

| Profile | Services | Namespace |
|---------|----------|-----------|
| core | PostgreSQL, Redis, MinIO | flexstack |
| observability | Prometheus, Grafana, Loki, Tempo | flexstack-observability |
| ai | LocalAI, AGiXT | flexstack-ai |
| identity | Keycloak, Vault | flexstack-identity |
| edge | Kong | flexstack-edge |
| messaging | NATS, Temporal | flexstack-messaging |

### Enable/Disable Profiles

```yaml
# Disable specific profiles
observability:
  enabled: false

identity:
  enabled: false
```

## Development

### Lint Charts

```bash
helm lint ./flexstack
```

### Template Charts

```bash
helm template flexstack ./flexstack --debug
```

### Test Installation

```bash
helm install flexstack ./flexstack --dry-run --debug
```

## Migrating from Docker Compose

See [Kubernetes Migration Guide](../docs/deployment/KUBERNETES_MIGRATION.md) for detailed instructions on migrating from Docker Compose to Kubernetes/Helm.

## Related Documentation

- [Kubernetes Manifests](../manifests/kubernetes/README.md)
- [Docker Compose](../docker/docker-compose.yml)
- [ArgoCD Integration](../manifests/argocd/applications.yaml)
