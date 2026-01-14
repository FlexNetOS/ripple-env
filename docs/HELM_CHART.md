# FlexStack Helm Chart Documentation

**Status:** Complete
**Last Updated:** 2026-01-14
**Location:** `charts/flexstack/`

---

## Overview

The FlexStack Helm chart deploys the complete agentic system platform to Kubernetes, including AI/ML services, observability, identity management, and workflow automation.

---

## Chart Metadata

| Field | Value |
|-------|-------|
| Name | `flexstack` |
| Version | `1.0.0` |
| App Version | `1.0.0` |
| Type | `application` |
| Home | https://github.com/FlexNetOS/ripple-env |

---

## Dependencies

The chart uses the following external Helm charts as dependencies:

| Dependency | Version | Repository | Condition |
|------------|---------|------------|-----------|
| postgresql | 15.x.x | https://charts.bitnami.com/bitnami | `postgresql.enabled` |
| redis | 19.x.x | https://charts.bitnami.com/bitnami | `redis.enabled` |
| minio | 14.x.x | https://charts.bitnami.com/bitnami | `minio.enabled` |
| grafana | 8.x.x | https://grafana.github.io/helm-charts | `observability.grafana.enabled` |
| prometheus | 25.x.x | https://prometheus-community.github.io/helm-charts | `observability.prometheus.enabled` |
| loki | 6.x.x | https://grafana.github.io/helm-charts | `observability.loki.enabled` |
| tempo | 1.x.x | https://grafana.github.io/helm-charts | `observability.tempo.enabled` |
| kong | 2.x.x | https://charts.konghq.com | `edge.kong.enabled` |
| nats | 1.x.x | https://nats-io.github.io/k8s/helm/charts | `messaging.nats.enabled` |

### Managing Dependencies

```bash
# Update dependency lock file
cd charts/flexstack
helm dependency update

# List current dependencies
helm dependency list

# Build dependencies locally
helm dependency build
```

---

## Installation

### Prerequisites

- Kubernetes 1.25+
- Helm 3.12+
- kubectl configured for target cluster
- Persistent volume provisioner (for storage)

### Quick Install

```bash
# Add required repositories
helm repo add bitnami https://charts.bitnami.com/bitnami
helm repo add grafana https://grafana.github.io/helm-charts
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm repo add kong https://charts.konghq.com
helm repo add nats https://nats-io.github.io/k8s/helm/charts
helm repo update

# Install FlexStack
helm install flexstack ./charts/flexstack \
  --namespace flexstack \
  --create-namespace
```

### Custom Values

```bash
# Install with custom values
helm install flexstack ./charts/flexstack \
  --namespace flexstack \
  --create-namespace \
  -f my-values.yaml

# Or override specific values
helm install flexstack ./charts/flexstack \
  --set global.domain=mycompany.com \
  --set postgresql.auth.password=secure-password
```

---

## Configuration

### Global Settings

```yaml
global:
  imagePullSecrets: []        # Registry credentials
  storageClass: ""            # Default storage class
  domain: "flexstack.local"   # Base domain for ingress
```

### Namespace Configuration

```yaml
namespaces:
  create: true
  core: flexstack
  observability: flexstack-observability
  ai: flexstack-ai
  identity: flexstack-identity
  edge: flexstack-edge
  messaging: flexstack-messaging
```

### Component Toggles

Enable/disable components as needed:

```yaml
# Core infrastructure (required)
postgresql:
  enabled: true
redis:
  enabled: true
minio:
  enabled: true

# Observability (recommended)
observability:
  enabled: true
  prometheus:
    enabled: true
  grafana:
    enabled: true
  loki:
    enabled: true
  tempo:
    enabled: true

# AI/ML services
ai:
  enabled: true
  localai:
    enabled: true
  agixt:
    enabled: true
  agixtUi:
    enabled: true

# Identity & Security
identity:
  enabled: true
  keycloak:
    enabled: true
  vault:
    enabled: true

# Data services
data:
  enabled: true
  neo4j:
    enabled: true

# Edge services
edge:
  enabled: true
  kong:
    enabled: true

# Messaging
messaging:
  enabled: true
  nats:
    enabled: true
  temporal:
    enabled: true

# Automation
automation:
  enabled: true
  n8n:
    enabled: true
  opa:
    enabled: true
```

---

## Resource Requirements

### Minimum Resources (Development)

| Component | Memory Request | Memory Limit | CPU Request | CPU Limit |
|-----------|---------------|--------------|-------------|-----------|
| PostgreSQL | 512Mi | 2Gi | 500m | 2 |
| Redis | 256Mi | 2Gi | - | - |
| MinIO | 512Mi | 2Gi | - | - |
| LocalAI | 4Gi | 8Gi | 2 | 4 |
| AGiXT | 1Gi | 4Gi | 500m | 4 |
| Keycloak | 1Gi | 2Gi | 500m | 2 |
| Neo4j | 1Gi | 2Gi | 500m | 2 |
| Temporal | 1Gi | 2Gi | 500m | 2 |
| Prometheus | 512Mi | 2Gi | 500m | 2 |
| Grafana | 256Mi | 1Gi | - | - |

### Storage Requirements

| Component | Default Size | Purpose |
|-----------|--------------|---------|
| PostgreSQL | 10Gi | Database storage |
| Redis | 5Gi | Cache persistence |
| MinIO | 50Gi | Object storage |
| LocalAI Models | 100Gi | AI model storage |
| Prometheus | 50Gi | Metrics retention |
| Loki | 50Gi | Log retention |
| Neo4j | 20Gi | Graph database |

---

## Profiles

### Minimal (Development)

```yaml
# minimal-values.yaml
postgresql:
  enabled: true
redis:
  enabled: true

observability:
  enabled: false

ai:
  enabled: true
  localai:
    enabled: true
  agixt:
    enabled: false

identity:
  enabled: false

edge:
  enabled: false

messaging:
  enabled: false
```

### Production

```yaml
# production-values.yaml
global:
  domain: "flexstack.example.com"

postgresql:
  enabled: true
  primary:
    persistence:
      size: 100Gi
  auth:
    password: "CHANGE_ME"

kong:
  replicaCount: 3

observability:
  prometheus:
    server:
      retention: 90d
      persistentVolume:
        size: 200Gi
```

---

## Upgrades

```bash
# Upgrade existing installation
helm upgrade flexstack ./charts/flexstack \
  --namespace flexstack \
  -f my-values.yaml

# Dry-run to preview changes
helm upgrade flexstack ./charts/flexstack \
  --namespace flexstack \
  --dry-run \
  --debug
```

---

## Uninstallation

```bash
# Uninstall chart (keeps PVCs)
helm uninstall flexstack --namespace flexstack

# Delete PVCs manually if needed
kubectl delete pvc -n flexstack --all

# Delete namespace
kubectl delete namespace flexstack
```

---

## Chart Structure

```
charts/flexstack/
├── Chart.yaml              # Chart metadata and dependencies
├── values.yaml             # Default configuration values
├── templates/
│   ├── _helpers.tpl        # Template helpers
│   ├── namespace.yaml      # Namespace definitions
│   ├── NOTES.txt          # Post-install notes
│   ├── ai/
│   │   ├── agixt.yaml     # AGiXT deployment
│   │   └── localai.yaml   # LocalAI deployment
│   └── messaging/
│       └── temporal.yaml  # Temporal deployment
└── (charts/)              # Downloaded dependencies (gitignored)
```

---

## Custom Templates

### LocalAI Deployment

Location: `templates/ai/localai.yaml`

Key features:
- AIO CPU image by default
- Configurable model storage
- Health checks enabled
- Resource limits defined

### AGiXT Deployment

Location: `templates/ai/agixt.yaml`

Key features:
- PostgreSQL integration
- MinIO S3 storage
- LocalAI as default provider

### Temporal Deployment

Location: `templates/messaging/temporal.yaml`

Key features:
- PostgreSQL backend
- Auto-setup mode
- UI service included

---

## Troubleshooting

### Dependency Errors

```bash
# Clear dependency cache
rm -rf charts/flexstack/charts/
rm charts/flexstack/Chart.lock

# Rebuild dependencies
helm dependency update charts/flexstack
```

### Installation Fails

```bash
# Check pod status
kubectl get pods -n flexstack

# Check events
kubectl get events -n flexstack --sort-by='.lastTimestamp'

# Check specific deployment
kubectl describe deployment flexstack-localai -n flexstack
```

### Storage Issues

```bash
# Check PVCs
kubectl get pvc -n flexstack

# Check storage class
kubectl get storageclass
```

---

## References

- [Helm Documentation](https://helm.sh/docs/)
- [values.yaml](../charts/flexstack/values.yaml) - Full configuration reference
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
- [PORTS.md](PORTS.md) - Port mappings
